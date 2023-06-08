#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Demonstrates how to stabilize the attitude tumble without translational motion.
This script sets up a 6-DOF spacecraft, but without specifying any orbital motion.  Thus,
this scenario simulates the spacecraft translating in deep space.  The scenario is a
version of :ref:`scenarioAttitudePointing` where the :ref:`mrpPD` feedback control
module is replaced with an equivalent python based BSK MRP PD control module.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudePointingPyDEPRECATED.py

As with :ref:`scenarioAttitudePointing`, when
the simulation completes 3 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.

This script showcases the deprecated way of creating Python modules.
For the newer way, refer to :ref:`scenarioAttitudePointingPy`.

The MRP PD control module in this script is a class called ``PythonMRPPD``.  Note that it has the
same setup and update routines as are found with a C/C++ Basilisk module.

To use a Python module in a simulation script, not that the python modules must be added to a special python specific
process and task list.  This is done with the commands::

    pyModulesProcess = scSim.CreateNewPythonProcess(pyProcessName, 9)
    pyModulesProcess.createPythonTask(pyTaskName, simulationTimeStep, True, -1)

Note that the python processes are always evaluated after the regular C/C++ processes.  Thus, the priority number
only controls the order of the python processes, not the python process execution relative to regular
Basilisk processes.

Creating an instance of the Python module is done with the code::

    pyMRPPD = PythonMRPPD("pyMRP_PD", True, 100)
    pyMRPPD.K = 3.5
    pyMRPPD.P = 30.0
    pyModulesProcess.addModelToTask(pyTaskName, pyMRPPD)

The first argument is the module tag string, the second is a bool argument specifying if the module is active
or note, and the last is the priority value for this module.  The next step is to configure the module
variables as you do with any other Basilisk module.  Finally, the module is added to the special task
list specifically for executing python modules.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

Here a small initial tumble is simulated.  The
resulting attitude and control torque histories are shown below.  The spacecraft quickly
regains a stable orientation without tumbling past 180 degrees.

.. image:: /_images/Scenarios/scenarioAttitudePointingPyDEPRECATED1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudePointingPyDEPRECATED2.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test showing how to setup and run a Python BSK module with C/C++ modules
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 16, 2021
#

import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import (
    unitTestSupport,
)  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import simulationArchTypes

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav

# import FSW Algorithm related support
# from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError

# import message declarations
from Basilisk.architecture import messaging

# attempt to import vizard
from Basilisk.utilities import vizSupport

# Used to deprecate the entire scenario or ignore certain deprecation warnings
from Basilisk.utilities import deprecated

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


@deprecated.deprecated(
    "2024/04/01",
    "The functionality showcased in this Scenario is deprecated."
    " See 'scenarioAttitudePointingPy.py'",
)
def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    #
    #  From here on scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"
    pyTaskName = "pyTask"
    pyProcessName = "pyProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.0)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName, 10)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # create the process and task that contains the Python modules
    # This context manager is used to hide the deprecate warning,
    # as we were expecting it
    with deprecated.ignore("CreateNewPythonProcess"):
        pyModulesProcess = scSim.CreateNewPythonProcess(pyProcessName, 9)
    pyModulesProcess.createPythonTask(pyTaskName, simulationTimeStep, True, -1)

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [
        [0.0],
        [0.0],
        [0.0],
    ]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0.0, 0.0, 0.0]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    # setup Python MRP PD control module
    pyMRPPD = PythonMRPPD("pyMRP_PD", True, 100)
    pyMRPPD.K = 3.5
    pyMRPPD.P = 30.0
    pyModulesProcess.addModelToTask(pyTaskName, pyMRPPD)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = unitTestSupport.samplingTime(
        simulationTime, simulationTimeStep, numDataPoints
    )
    attErrorLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    mrpLog = pyMRPPD.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    pyMRPPD.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(pyMRPPD.cmdTorqueOutMsg)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(
        scSim,
        simTaskName,
        scObject
        # , saveFile=fileName
    )

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    timeAxis = attErrorLog.times()
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(
            timeAxis * macros.NANO2MIN,
            dataSigmaBR[:, idx],
            color=unitTestSupport.getLineColor(idx, 3),
            label=r"$\sigma_" + str(idx) + "$",
        )
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel(r"Attitude Error $\sigma_{B/R}$")
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(
            timeAxis * macros.NANO2MIN,
            dataLr[:, idx],
            color=unitTestSupport.getLineColor(idx, 3),
            label="$L_{r," + str(idx) + "}$",
        )
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("Control Torque $L_r$ [Nm]")
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(
            timeAxis * macros.NANO2MIN,
            dataOmegaBR[:, idx],
            color=unitTestSupport.getLineColor(idx, 3),
            label=r"$\omega_{BR," + str(idx) + "}$",
        )
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("Rate Tracking Error [rad/s] ")

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


class PythonMRPPD(simulationArchTypes.PythonModelClass):
    """
    This class inherits from the `PythonModelClass` available in the ``simulationArchTypes`` module.
    The `PythonModelClass` is the parent class which your Python BSK modules must inherit.
    The class uses the following
    virtual functions:

    #. ``reset``: The method that will initialize any persistent data in your model to a common
       "ready to run" state (e.g. filter states, integral control sums, etc).
    #. ``updateState``: The method that will be called at the rate specified
       in the PythonTask that was created in the input file.

    Additionally, your class should ensure that in the ``__init__`` method, your call the super
    ``__init__`` method for the class so that the base class' constructor also gets called to
    initialize the model-name, activity, moduleID, and other important class members:

    .. code-block:: python

        super(PythonMRPPD, self).__init__(modelName, modelActive, modelPriority)

    You class must implement the above four functions. Beyond these four functions you class
    can complete any other computations you need (``Numpy``, ``matplotlib``, vision processing
    AI, whatever).
    """

    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        super(PythonMRPPD, self).__init__(modelName, modelActive, modelPriority)

        # Proportional gain term used in control
        self.K = 0
        # Derivative gain term used in control
        self.P = 0
        # Input guidance structure message
        self.guidInMsg = messaging.AttGuidMsgReader()
        # Output body torque message name
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()

    def reset(self, currentTime):
        """
        The reset method is used to clear out any persistent variables that need to get changed
        when a task is restarted.  This method is typically only called once after selfInit/crossInit,
        but it should be written to allow the user to call it multiple times if necessary.
        :param currentTime: current simulation time in nano-seconds
        :return: none
        """
        return

    def updateState(self, currentTime):
        """
        The updateState method is the cyclical worker method for a given Basilisk class.  It
        will get called periodically at the rate specified in the Python task that the model is
        attached to.  It persists and anything can be done inside of it.  If you have realtime
        requirements though, be careful about how much processing you put into a Python updateState
        method.  You could easily detonate your sim's ability to run in realtime.

        :param currentTime: current simulation time in nano-seconds
        :return: none
        """
        # read input message
        guidMsgBuffer = self.guidInMsg()

        # create output message buffer
        torqueOutMsgBuffer = messaging.CmdTorqueBodyMsgPayload()

        # compute control solution
        lrCmd = (
            np.array(guidMsgBuffer.sigma_BR) * self.K
            + np.array(guidMsgBuffer.omega_BR_B) * self.P
        )
        torqueOutMsgBuffer.torqueRequestBody = (-lrCmd).tolist()

        self.cmdTorqueOutMsg.write(torqueOutMsgBuffer, currentTime, self.moduleID)

        def print_output():
            """Sample Python module method"""
            print(currentTime * 1.0e-9)
            print(torqueOutMsgBuffer.torqueRequestBody)
            print(guidMsgBuffer.sigma_BR)
            print(guidMsgBuffer.omega_BR_B)

        return


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(True)  # show_plots
