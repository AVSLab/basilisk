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

This BSK-Sim scenario demonstrates how a Delta-V maneuver can be performed using thrusters. A few minutes before the
maneuver, the attitude of the spacecraft is brought to the DV attitude, i.e. the DV thrusters of the spacecraft are
aligned with the DV direction. This is done using the reaction wheels and the modules :ref:`dvGuidance`,
:ref:`mrpFeedback` and :ref:`rwMotorTorque`. During the maneuver, the attitude os controlled with the DV thrusters and
the modules :ref:`dvGuidance`, :ref:`mrpFeedback`, :ref:`thrForceMapping` and :ref:`thrFiringRemainder`. The DV
maneuver is executed using the :ref:`dvExecuteGuidance` module. The maneuver starts at a specified burn start time and
ends once the desired DV has been accumulated.

The script is found in the folder ``basilisk/examples/BskSim/scenarios`` and executed by using::

      python3 scenario_ClosedLoopManeuver.py


Custom Dynamics Configurations Instructions
-------------------------------------------

Delta-V thrusters were added to the :ref:`BSK_Dynamics` framework. The properties of the DV thrusters can be changed
there, but keep in mind to also change the thruster properties in :ref:`BSK_FSW`.


Custom FSW Configurations Instructions
--------------------------------------

Delta-V thrusters were added to the :ref:`BSK_FSW` framework. The properties of the DV thrusters can be changed
there, but keep in mind to also change the thruster properties in :ref:`BSK_Dynamics`. Make sure to call
`FswModel.SetAttThrusters(useDvThrusters = True)` if it is desired to use the DV thrusters for attitude control,
otherwise the ACS thrusters will be used by default.

The necessary modules for attitude control with thrusters, i.e. :ref:`thrForceMapping` and :ref:`thrFiringRemainder`
were added, and the module properties can be changed there. An additional MRP feedback module was added for thrusters.
Modules for the DV execution, i.e. :ref:`dvGuidance` for DV attitude control and :ref:`dvExecuteGuidance` for maneuver
execution were also added and can be adjusted in :ref:`BSK_FSW`.

Illustration of Simulation Results
----------------------------------

::

    showPlots = True

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_attitudeErrorNorm.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_attitudeRateError.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_controlTorque.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_dvOrientation.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_dvCommand.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_dvAccumulation.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_thrusterOnTime.svg
   :align: center

.. image:: /_images/Scenarios/scenario_ClosedLoopManeuver_thrusterForce.svg
   :align: center

"""



# Get current file path
import inspect
import os
import sys

import numpy as np
# Import utilities
from Basilisk.utilities import orbitalMotion, macros, vizSupport, RigidBodyKinematics, unitTestSupport
from Basilisk.architecture import messaging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/..')
from BSK_masters import BSKSim, BSKScenario
import BSK_Dynamics, BSK_Fsw

# Import plotting files for your scenario
sys.path.append(path + '/../plotting')
import BSK_Plotting as BSK_plt

# Create your own scenario child class
class scenario_ClosedLoopManeuver(BSKSim, BSKScenario):
    def __init__(self):
        super(scenario_ClosedLoopManeuver, self).__init__(fswRate=1., dynRate=0.1)
        self.name = 'scenario_ClosedLoopManeuver'

        # declare additional class variables
        self.attNavRec = None
        self.transNavRec = None
        self.thrusterDynRec = None
        self.attErrRec = None
        self.attRefRec = None
        self.LrRec = None
        self.dvRec = None
        self.thrusterFswRec = None

        self.set_DynModel(BSK_Dynamics)
        self.set_FswModel(BSK_Fsw)

        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()

        # we want to align SC body z-axis with Delta-V direction, correct reference frame
        dcm_BcB = np.array([[0., 0., 1.],
                            [0., 1., 0.],
                            [-1., 0., 0.]])
        sigma_BcB = RigidBodyKinematics.C2MRP(dcm_BcB)
        FswModel.attRefCorrection.sigma_BcB = sigma_BcB

        tManeuver = 15.
        # use stand-alone message to write DvBurnCmdMsg
        dvBurnCmdMsgData = messaging.DvBurnCmdMsgPayload()
        dvBurnCmdMsgData.dvInrtlCmd = [10., 100., 20.]
        dvBurnCmdMsgData.dvRotVecUnit = [1., 0., 0.]
        dvBurnCmdMsgData.burnStartTime = macros.min2nano(tManeuver)
        FswModel.dvBurnCmdMsg.write(dvBurnCmdMsgData)

        # configure Lambert modules to write DvBurnCmdMsg.
        # Only used if the lines to activate the Lambert flight mode are uncommented
        r_TN_N = np.array([(42164 * 1e3), 0., 0.])  # targeted position
        tFinal = 120.
        FswModel.lambertPlannerObject.r_TN_N = r_TN_N
        FswModel.lambertPlannerObject.finalTime = tFinal * 60.
        FswModel.lambertPlannerObject.maneuverTime = tManeuver * 60.
        FswModel.lambertPlannerObject.mu = DynModel.gravFactory.gravBodies['earth'].mu
        FswModel.lambertPlannerObject.numRevolutions = 0
        FswModel.lambertValidatorObject.finalTime = tFinal * 60.
        FswModel.lambertValidatorObject.maneuverTime = tManeuver * 60.
        FswModel.lambertValidatorObject.ignoreConstraintViolations = True

        # use the DV thrusters for attitude control instead of ACS thrusters
        FswModel.SetAttThrusters(True)

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        DynModel = self.get_DynModel()
        vizSupport.enableUnityVisualization(self, DynModel.taskName, DynModel.scObject
                                            # , saveFile=__file__
                                            , thrEffectorList=DynModel.thrustersDynamicEffectorDV
                                            , rwEffectorList=DynModel.rwStateEffector
                                            )

    def configure_initial_conditions(self):
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 42000. * 1e3  # meters
        oe.e = 0.001
        oe.i = 1. * macros.D2R
        oe.Omega = 1. * macros.D2R
        oe.omega = 1. * macros.D2R
        oe.f = -30. * macros.D2R
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    def log_outputs(self):
        FswModel = self.get_FswModel()
        DynModel = self.get_DynModel()
        samplingTime = int(FswModel.processTasksTimeStep/10)
        # Dynamics process outputs
        self.attNavRec = DynModel.simpleNavObject.attOutMsg.recorder(samplingTime)
        self.transNavRec = DynModel.simpleNavObject.transOutMsg.recorder(samplingTime)
        self.thrusterDynRec = []
        self.numThrusters = len(DynModel.thrustersDynamicEffectorDV.thrusterOutMsgs)
        for item in range(0, self.numThrusters):
            self.thrusterDynRec.append(DynModel.thrustersDynamicEffectorDV.thrusterOutMsgs[item].recorder(samplingTime))

        # FSW process outputs
        self.attRefRec = FswModel.attRefMsg.recorder(samplingTime)
        self.attErrRec = FswModel.attGuidMsg.recorder(samplingTime)
        self.LrRec = FswModel.cmdTorqueMsg.recorder(samplingTime)
        self.dvRec = FswModel.dvBurnCmdMsg.recorder(samplingTime)
        self.thrusterFswRec = FswModel.dvOnTimeCmdMsg.recorder(samplingTime)

        self.AddModelToTask(DynModel.taskName, self.attNavRec)
        self.AddModelToTask(DynModel.taskName, self.transNavRec)
        for item in range(self.numThrusters):
            self.AddModelToTask(DynModel.taskName, self.thrusterDynRec[item])
        self.AddModelToTask(DynModel.taskName, self.attRefRec)
        self.AddModelToTask(DynModel.taskName, self.attErrRec)
        self.AddModelToTask(DynModel.taskName, self.LrRec)
        self.AddModelToTask(DynModel.taskName, self.dvRec)
        self.AddModelToTask(DynModel.taskName, self.thrusterFswRec)

    def pull_outputs(self, showPlots):
        # Dynamics process outputs
        sigma_BN = np.delete(self.attNavRec.sigma_BN, 0, 0)
        r_BN_N = np.delete(self.transNavRec.r_BN_N, 0, 0)
        dvAccum = np.delete(self.transNavRec.vehAccumDV, 0, 0)
        thrustersDyn = np.empty([len(r_BN_N), self.numThrusters])
        for i in range(0, self.numThrusters):
            thrustersDyn[:, i] = np.delete(self.thrusterDynRec[i].thrustForce, 0, 0)

        # FSW process outputs
        sigma_BR = np.delete(self.attErrRec.sigma_BR, 0, 0)
        omega_BR_B = np.delete(self.attErrRec.omega_BR_B, 0, 0)
        Lr = np.delete(self.LrRec.torqueRequestBody, 0, 0)
        dvCmd = np.delete(self.dvRec.dvInrtlCmd, 0, 0)
        thrustersOnTime = np.delete(self.thrusterFswRec.OnTimeRequest, 0, 0)

        # Plot results
        BSK_plt.clear_all_plots()
        timeLineSet = np.delete(self.attNavRec.times(), 0, 0) * macros.NANO2MIN
        BSK_plt.plot_attitude_error(timeLineSet, sigma_BR)
        BSK_plt.plot_rate_error(timeLineSet, omega_BR_B)
        BSK_plt.plot_control_torque(timeLineSet, Lr)
        BSK_plt.plot_dv_orientation(timeLineSet, r_BN_N, dvCmd, sigma_BN)
        BSK_plt.plot_dv_command(timeLineSet, dvCmd)
        BSK_plt.plot_dv_accumulation(timeLineSet, dvAccum, sigma_BN)
        BSK_plt.plot_thrOnTime(timeLineSet, thrustersOnTime, self.numThrusters)
        BSK_plt.plot_thrForce(timeLineSet, thrustersDyn, self.numThrusters)

        figureList = {}
        if showPlots:
            BSK_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["attitudeErrorNorm", "attitudeRateError", "controlTorque", "dvOrientation", "dvCommand",
                           "dvAccumulation", "thrusterOnTime", "thrusterForce"]
            figureList = BSK_plt.save_all_plots(fileName, figureNames)

        return figureList

def runScenario(TheScenario):

    # Initialize simulation
    TheScenario.InitializeSimulation()

    dvBurnModeEnd = 25.

    # Configure run time and execute simulation

    # if the Lambert modules should be used to write the DvBurnCmdMsg (instead of using a stand-alone message),
    # uncomment the following lines

    # simulationTime = macros.min2nano(2.)
    # TheScenario.modeRequest = 'lambertFirstDV'
    # TheScenario.ConfigureStopTime(simulationTime)
    # TheScenario.ExecuteSimulation()
    # dvBurnModeEnd = 50.

    simulationTime = macros.min2nano(15.)
    TheScenario.modeRequest = 'dvPoint'
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()

    simulationTime = macros.min2nano(dvBurnModeEnd)
    TheScenario.modeRequest = 'dvBurn'
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()

    simulationTime = macros.min2nano(60.)
    TheScenario.modeRequest = 'dvPoint'
    TheScenario.ConfigureStopTime(simulationTime)
    TheScenario.ExecuteSimulation()


def run(showPlots):
    """
        The scenarios can be run with the followings setups parameters:

        Args:
            showPlots (bool): Determines if the script should display plots

    """
    # Instantiate base simulation
    scenario = scenario_ClosedLoopManeuver()
    runScenario(scenario)
    figureList = scenario.pull_outputs(showPlots)
    return figureList


if __name__ == "__main__":
    run(True)
