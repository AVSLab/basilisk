#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
r"""
Overview
--------

The goal of the OpNav Scenarios is to simulate Optical Navigation methods and performance in Basilisk.
All of the scenarios provided in ``basilisk/examples/OpNavScenarios`` put a spacecraft on orbit about Mars. By extracting
limbs are circles from the images, the spacecraft can point to the planet, and estimate it's position.

.. image:: /_images/static/OpNavScenario.png
   :align: center

This Basilisk Simulation, which inherits ``SimulationBaseClass``, provides the backbone for all the OpNav simulations
provided in ``basilisk/examples/OpNavScenarios``.
These simulations spawn the Basilisk :ref:`Vizard <Vizard>` visualization in order to provide images for processing.
These images are
handled by the vizInterface module found in ``src/simulation/vizInterface``. A figure illustrating the architecture
is found here:

.. image:: /_images/static/OpNavInterfaceOverview.png
   :scale: 50%
   :align: center

More details on the software interaction can be found in Chapter 2 of `Thibaud Teil's PhD thesis <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.
Sequentially, Basilisk modules then receive the images in order to process and navigate using them. This is illustrated
in more detail:

.. image:: /_images/static/OpNav_Details.png
   :scale: 50%
   :align: center


Running OpNav Simulations
-------------------------
.. important::
    Make sure Basilisk is built to include the OpNav modules by using ``--opNav True`` when running ``conanfile.py``.
    This is not turned on by default as it will install the large OpenCL library.

In order to call Vizard from python simulations, the path to the downloaded Vizard app must be properly set.
If needed, adjust the following line illustrated below to reflect the path to the Vizard binary::

    # TODO : Modify the path to the viz here
    appPath = '/Applications/Vizard.app/Contents/MacOS/Vizard' #If on Mac

The Vizard app must therefore me downloaded, and this path must reflect it's position in the file structure, and its
name. If the path is not properly set, the OpNav simulations will hang (printing that it is waiting for the
Vizard connection). Another option is to manually open the Vizard application after having started the python scenario,
check OpNav or Direct Comm, and provide the tcp/ip address printed by the scenario.

The scripts are tested if all modules are installed, but can be run at full length by calling::

    python3 scenario_OpNavAttOD.py

See the Scene Rendering
-----------------------
By default the Vizard will run in head-less mode where the full scene is note rendered
for viewing as this slows down the simulation.  To see the scene, edit the following line
within the scenario script::

    TheBSKSim.get_DynModel().vizInterface.noDisplay = False

to set ``noDisplay`` flag to False instead of True.  Further, 2-way communication is
enabled by adding the line::

    TheBSKSim.get_DynModel().vizInterface.liveStream = True

Finally, the Vizard binary is called with the ``-directComm`` argument using::

    TheScenario.run_vizard("-directComm")


OpNav Dynamics, Flight Software, and Plotting
---------------------------------------------

The simulations use three other main python scripts.

:ref:`BSK_OpNavDynamics` is similar to the BSKSim versions seen previously. The main additions are
the instantiation of :ref:`vizInterface`, and the camera module.

:ref:`BSK_OpNavFsw` contains the FSW algorithms used in the scenarios. Examples are the Orbit Determination
filters, the pointing guidance module, the CNN module, and more. This file also contains the ``modeRequest``
definitions which enable all the tasks necessary to perform a specific action.

:ref:`OpNav_Plotting` contains the plotting routines. None of the files are saved, but are shown when
the scenario is run with python. Saving is left to the user's discretion.

"""


# Get current file path
import inspect
import os
import subprocess
import sys

# Import architectural modules
from Basilisk.utilities import SimulationBaseClass

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import Dynamics and FSW models
sys.path.append(path + '/modelsOpNav')

# TODO : Modify the path to the viz here
appPath = '/Applications/Vizard.app/Contents/MacOS/Vizard' #If on Mac
# appPath = './../../Applications/Vizard.app' #If on Linux


class BSKSim(SimulationBaseClass.SimBaseClass):
    """
    BSK Simulation base class for opNav scenarios
    """
    def __init__(self, fswRate=0.1, dynRate=0.1):
        self.dynRate = dynRate
        self.fswRate = fswRate
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        self.SetProgressBar(True)

        self.vizPath = appPath
        self.DynModels = []
        self.FSWModels = []
        self.DynamicsProcessName = None
        self.dynProc = None
        self.FSWProcessName = None
        self.fswProc = None

        self.dynamics_added = False
        self.fsw_added = False

    def get_DynModel(self):
        assert (self.dynamics_added is True), "It is mandatory to use a dynamics model as an argument"
        return self.DynModels

    def set_DynModel(self, dynModel):
        self.dynamics_added = True
        self.DynamicsProcessName = 'DynamicsProcess'  # Create simulation process name
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName, 100)  # Create process
        self.DynModels = dynModel.BSKDynamicModels(self, self.dynRate)  # Create Dynamics and FSW classes

    def get_FswModel(self):
        assert (self.fsw_added is True), "A flight software model has not been added yet"
        return self.FSWModels

    def set_FswModel(self, fswModel):
        self.fsw_added = True
        self.FSWProcessName = "FSWProcess"  # Create simulation process name
        self.fswProc = self.CreateNewProcess(self.FSWProcessName, 10)  # Create process
        self.FSWModels = fswModel.BSKFswModels(self, self.fswRate)  # Create Dynamics and FSW classes


class BSKScenario(object):
    def __init__(self, masterSim, showPlots):
        self.name = "scenario"
        self.masterSim = masterSim
        self.vizard = None

        self.showPlots = showPlots

    def configure_initial_conditions(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass

    def log_outputs(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass

    def pull_outputs(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass
    def run_vizard(self, mode):
        try:
            self.vizard = subprocess.Popen(
                [self.masterSim.vizPath, "--args", mode, "tcp://localhost:5556"], stdout=subprocess.DEVNULL)

            print("Vizard spawned with PID = " + str(self.vizard.pid))
        except FileNotFoundError:
            print("Vizard application not found")
            if sys.platform != "darwin":
                print(
                    "Either download Vizard at this path %s or change appPath in BSK_OpNav.py file" % self.masterSim.vizPath)
            else:
                print("1. Download  Vizard interface \n2. Move it to Applications \n"
                      "3. Change only application name while initializing appPath variable in BSK_OpNav")
            exit(1)
    def end_scenario(self):

        if self.vizard is None:
            print("vizard application is not launched")
            exit(1)
        self.vizard.kill()

        # Pull the results of the base simulation running the chosen scenario
        figureList = {}
        if self.showPlots:
            figureList = self.pull_outputs(self.showPlots)
        return figureList
