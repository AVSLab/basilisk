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

"""

.. raw:: html

    <iframe width="560" height="315" src="https://www.youtube.com/embed/YjRM_3MOjAc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Overview
--------

This script sets up a 3-DOF spacecraft which is orbiting the Earth. The goal of the scenario is to

#. highlight the structure of the BSK_Sim architecture,
#. demonstrate how to create a custom BSK_scenario, and
#. how to customize the ``BSK_Dynamics.py`` and ``BSK_FSW.py`` files.

The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_BasicOrbit.py

The simulation mimics the basic simulation scenario in the earlier tutorial in
:ref:`scenarioBasicOrbit`.  But rather than explicitly defining all simulation properties
within the python simulation file, the bskSim spacecraft simulation class is used to encapsulate a lot of the
setup and configuring.

The simulation layout is shown in the following illustration.

.. image:: /_images/static/test_scenario_basicOrbit_v1.1.svg
   :align: center

Two simulation processes are created: one
which contains dynamics modules, and one that contains the Flight Software (FSW)
modules. The benefit of the new ``BSK_Sim`` architecture is how it allows the user to have a pre-written spacecraft
configurations and FSW modes neatly organized within three modular files: a ``BSK_scenario`` file, a FSW file, and
a Dynamics file.

More explicitly, the purpose of the scenario file (in this case :ref:`scenario_BasicOrbit`)
within the ``BSK_Simulation`` architecture is to provide the user a
simple, front-end interface to configure a scenario without having to individually initialize and integrate each
dynamics and FSW module into their simulation. Instead the Dynamics file
(for instance :ref:`BSK_Dynamics` or :ref:`BSK_FormationDynamics`)
has preconfigured many dynamics modules, attached them to the spacecraft,
and linked their messages to the appropriate FSW modules.
Similarly, the FSW file (in this case :ref:`BSK_FSW`) creates preconfigured FSW modes such as hill pointing, sun safe
pointing, velocity pointing, and more. Each preconfigured mode triggers a specific event which enables various FSW tasks
like assigning enabling a specific pointing model or control loop. The proceeding sequence of tasks then initialize the
appropriate FSW modules, link their messages, and provide pre-written FSW functionality through a simple
modeRequest variable within the BSK_scenario file.

Configuring the scenario file
-----------------------------

To write a custom scenario file, first create a class such as ``scenario_BasicOrbit`` that will
inherent from the masterSim class.
Following the inheritance, there are three functions within the scenario class that need to be defined by the user:
``configure_initial_conditions()``, ``log_outputs()``, and ``pull_outputs()``.

Within ``configure_initial_conditions()``, the user needs to define the spacecraft FSW
mode for the simulation through the ``modeRequest`` variable.
this is the parameter that triggers the aforementioned FSW event. Additional FSW modes (to be discussed in later
tutorials) include sunSafePoint, inertial3D, velocityPoint, hillPoint, and more.

Additionally, the user needs to supply initial conditions
for the spacecraft and its orbit. The example script code uses the :ref:`orbitalMotion` module to
construct the appropriate position and velocity vectors for a stable orbit, and then assigns them to the
spacecraft.

The ``self.masterSim.get_DynModel()`` is referencing a list of available dynamic modules preconfigured in the Dynamics file.

Within ``log_outputs()``, the user can supply a list of messages they are interested in logging. Position and velocity
from the navigation message are relevant to verify proper orbit functionality.

Finally within the ``pull_outputs()``, the user can pull specific variables from the logged messages
and proceed to graph them using predefined plotting routines in BSK_Plotting.py

Custom Configurations Instructions
----------------------------------

The benefit of the BSK_Simulation architecture is its user simplicity. Things like spacecraft hub configurations,
reaction wheel pyramids, and
coarse sun sensor constellations are all preconfigured; however, for users who would like to customize their own
dynamics modules and FSW modes, it is recommended to copy the two primary ``BSK_Sim`` files
(:ref:`BSK_Dynamics.py <BSK_Dynamics>` and :ref:`BSK_FSW.py <BSK_FSW>`) and modify them directly.
Instructions for configuring
user-customized Dynamics and FSW files are detailed below.


Custom Dynamics Configurations Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In :ref:`BSK_Dynamics`, the script first generates a dynamics task onto which
future dynamics modules will be added.
Following the task generation, all desired dynamics module objects are generated:
These objects are then configured through ``InitAllDynObjects(SimBase)`` which iterates through a number of setter
functions that configure all of the dynamics objects properties and messages.
These setter functions are examples of how the ``BSK_Sim`` architecture has preconfigured
dynamics modules within the :ref:`BSK_Dynamics`.
Now, for every future scenario file, a spacecraft object, gravity effector, and simple
navigation sensor will be available for use.
Finally, all now-configured objects are attached to the DynamicsTask


The number at the end of ``AddModelToTask`` corresponds with the priority of the model.
The higher the number, the earlier the model gets evaluated during each time step.


Custom FSW Configurations Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:ref:`BSK_FSW.py <BSK_FSW>`'s ``__init__()`` procedure defines all
possible configuration messages to be used by future FSW algorithms.
Because this scenario is simulating a 3-DOF spacecraft, there are no FSW algorithms needed to control attitude.

As such, a ``initializeStandby`` event is created within :ref:`BSK_FSW` to ensure all
FSW tasks are disabled. This event is
triggered by the modeRequest called in :ref:`scenario_BasicOrbit` and
executes the following code in :ref:`BSK_FSW`.


Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_BasicOrbit_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenario_BasicOrbit_orientation.svg
   :align: center


"""

# Get current file path
import inspect
import os
import sys

# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics
import BSK_Fsw

# Import plotting files for your scenario
import BSK_Plotting as BSK_plt


# Create your own scenario child class
class scenario_BasicOrbit(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_BasicOrbit, self).__init__()
        self.name = 'scenario_BasicOrbit'

        # declare empty class variables
        self.sNavAttRec = None
        self.sNavTransRec = None

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynModels.scObject
                                            # , saveFile=__file__
                                            , rwEffectorList=self.DynModels.rwStateEffector
                                            )

    def configure_initial_conditions(self):
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 7000000.0  # meters
        oe.e = 0.1
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        # Dynamics process outputs
        DynModels = self.get_DynModel()
        self.sNavAttRec = DynModels.simpleNavObject.attOutMsg.recorder()
        self.sNavTransRec = DynModels.simpleNavObject.transOutMsg.recorder()
        self.AddModelToTask(DynModels.taskName, self.sNavAttRec)
        self.AddModelToTask(DynModels.taskName, self.sNavTransRec)

    def pull_outputs(self, showPlots):
        # Dynamics process outputs
        sigma_BN = self.sNavAttRec.sigma_BN
        r_BN_N = self.sNavTransRec.r_BN_N
        v_BN_N = self.sNavTransRec.v_BN_N

        # Plot results
        BSK_plt.clear_all_plots()
        timeLineSet = self.sNavAttRec.times() * macros.NANO2MIN
        BSK_plt.plot_orbit(r_BN_N)
        BSK_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "orientation"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(scenario):

    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure FSW mode
    scenario.modeRequest = 'standby'

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()


def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_BasicOrbit()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
