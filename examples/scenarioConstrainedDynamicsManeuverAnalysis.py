# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Overview
--------

This scenario further demonstrates the capabilities of :ref:`constraintDynamicEffector` in simulating a dynamic coupling
between two spacecraft. The constraint effector allows the simulation of two spacecraft rigid hubs attached
through an arm of variable rigidity and is agnostic to the parameters of either vehicle. The scenario analyzes the
impact of different gains on the motion of two spacecraft while one spacecraft is performing a maneuver towing the
other. This scenario demonstrates a required analysis for any mission simulation utilizing the constraint effector, as
the gains must be tuned to achieve a desired fidelity and runtime dependent on how "dynamic" the scenario is stressing
the constraint. Aspects affecting this include:

* the vehicle mass and inertia
* vehicle spin rate
* the moment arm between spacecraft (connection point distance from COM and length of arm)
* accelerations acting on each spacecraft (gravity, thrust, disturbances)

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioConstrainedDynamicsManeuverAnalysis.py

The scenario outputs three plots. The first one shows the direction constraint violations vs. time for different
gains to show how they change for different arm stiffness values. The second plots shows the same time history but
for attitude constraint violations. The third plot shows the overall simulation performance, plotting both the
average constraint violations and simulation runtime vs. gain.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, gain_list = [1E1, 1E2, 1E3], relpos_config = 'alongtrackahead', orbit_config = 'LEO', maneuver_config = 'attitude', sc_model = 'MEV2'

The time history for the direction and attitude constraint violations during an attitude maneuver is shown
below for different values of the gains. The attitude maneuver controls the chaser to turn 180 degrees,
rotating the target along with it.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisattitudemaneuver_DirectionConstraint.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisattitudemaneuver_AttitudeConstraint.svg
   :align: center

The figure below shows the performance of the simulation for different gain values. The left axis shows the
constraint violations averaged over the entire simulation, while the right axis shows the total runtime of
the simulation. The dashed line is the attitude average and the solid line is the direction average.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisattitudemaneuver_GainPerformance.svg
   :align: center

Performing the same analysis for an orbital maneuver produces the following results. The orbital maneuver
controls the chaser to push the stack with the target ahead.

::

    show_plots = True, gain_list = [1E1, 1E2, 1E3], relpos_config = 'alongtrackahead', orbit_config = 'LEO', maneuver_config = 'orbit', sc_model = 'MEV2'

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisorbitmaneuver_DirectionConstraint.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisorbitmaneuver_AttitudeConstraint.svg
   :align: center

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsManeuverAnalysisorbitmaneuver_GainPerformance.svg
   :align: center

"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Illustrates coupled spacecraft maneuvering.
#   Author:             Andrew Morell
#   Creation Date:      Oct 4, 2025
#

# Basilisk imports
from Basilisk.architecture import messaging
from Basilisk.utilities import (SimulationBaseClass, orbitalMotion, macros, RigidBodyKinematics, vizSupport)
from Basilisk.simulation import (spacecraft, constraintDynamicEffector, gravityEffector, svIntegrators, extForceTorque, simpleNav)
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError)
# plotting imports
import matplotlib.pyplot as plt
# utility imports
import numpy as np
import time

from Basilisk import __path__
import os
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


class SimBaseClass(SimulationBaseClass.SimBaseClass):
    def __init__(self, dynRate=0.1):
        self.dynRateSec = dynRate
        self.dynRateNanos = macros.sec2nano(dynRate)
        SimulationBaseClass.SimBaseClass.__init__(self)

        # Create simulation variable names
        self.simTaskName = "simTask"
        simProcessName = "simProcess"

        # Create the dynamics task and set the integration update time
        self.dynProcess = self.CreateNewProcess(simProcessName)
        self.dynProcess.addTask(self.CreateNewTask(self.simTaskName, self.dynRateNanos))


def run(show_plots, gain_list, relpos_config, orbit_config, maneuver_config, sc_model):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
        gain_list (float list): Choose which gain values to test (how stiff the arm is, ideally 1E1 to 1E4)
        relpos_config (str): Choose where the servicer spacecraft starts w.r.t. the target ``alongtrackahead``,
            ``alongtrackbehind``, ``radial``, or ``antiradial``
        orbit_config (str): Choose which orbit configuration to use between ``LEO`` or ``no orbit``
        maneuver_config (str): Choose which maneuver configuration to use between ``orbit`` or ``attitude``
        sc_model (str): Choose which spacecraft models to use, determines mass/inertia properties  ``bskSat``, ``MEV1``, or ``MEV2``
    """

    scSim_list = []
    for gain in gain_list: # loop through gain
        scSim = SimBaseClass()
        create_spacecraft(scSim, sc_model)
        define_initial_conditions(scSim, orbit_config, relpos_config)
        set_up_constraint_effector(scSim, gain)
        set_up_maneuver(scSim, maneuver_config)
        log_data(scSim)

        set_up_vizard(scSim)
        run_simulation(scSim, maneuver_config)
        process_data(scSim)

        scSim_list.append(scSim)

    return plotting(scSim_list, gain_list, maneuver_config, show_plots)


def create_spacecraft(scSim, sc_model):
    # Create spacecraft
    scSim.scObject1 = spacecraft.Spacecraft()  # chaser spacecraft
    scSim.scObject1.ModelTag = "chaser"
    scSim.scObject2 = spacecraft.Spacecraft()  # target spacecraft
    scSim.scObject2.ModelTag = "target"

    # Add test modules to runtime call list
    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject2)

    # Set the servicer spacecraft integrator to RKF45
    integratorObject = svIntegrators.svIntegratorRKF45(scSim.scObject1)
    scSim.scObject1.setIntegrator(integratorObject)
    integratorObject.this.disown()

    # Sync dynamics integration of target spacecraft to dynamics integration of servicer spacecraft
    scSim.scObject1.syncDynamicsIntegration(scSim.scObject2)

    # Define mass properties of the rigid hub of both spacecraft
    if sc_model == 'bskSat':  # uses two identical "bskSats" which can be found in other Basilisk example scripts
        scSim.scObject1.hub.mHub = 750.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 750.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 0.1  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.1  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.1  # [m] arm_length |r_P2P1|

    elif sc_model == 'MEV1':  # chaser is MEV-1 and target is Intelsat-901
        scSim.scObject1.hub.mHub = 2330.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 4723.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 1.0  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.5  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.5  # [m] arm_length |r_P2P1|

    elif sc_model == 'MEV2':  # chaser is MEV-2 and target is Galaxy-30
        scSim.scObject1.hub.mHub = 2875.0  # [kg] chaser spacecraft mass
        scSim.scObject1.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B1 relative to chaser COM
        scSim.scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.scObject2.hub.mHub = 3298.0  # [kg] target spacecraft mass
        scSim.scObject2.hub.r_BcB_B = [[0.0], [0.0],
                                       [0.0]]  # [m] position vector of body-fixed point B2 relative to target COM
        scSim.scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                           [0.0, 0.0, 600.0]]  # [kg-m^2] chaser spacecraft inertia
        scSim.r_P1B1_mag = 1.0  # [m] position vector magnitude from chaser COM to connection point
        scSim.r_P2B2_mag = 0.5  # [m] position vector magnitude from target COM to connection point
        scSim.r_P2P1_mag = 0.5  # [m] arm_length |r_P2P1|

    else:
        raise Exception("sc_model must be 'bskSat', 'MEV1', or 'MEV2'")


def define_initial_conditions(scSim, orbit_config, relpos_config):
    # Note: here we assume that the N, B1 and B2 frames are identical, therefore no frame is defined
    if orbit_config == 'LEO':  # low Earth orbit
        earthGravBody = gravityEffector.GravBodyData()
        earthGravBody.planetName = "earth_planet_data"
        earthGravBody.mu = 0.3986004415E+15  # [meters^3/s^2]
        earthGravBody.isCentralBody = True
        scSim.scObject1.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])
        scSim.scObject2.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

        oe = orbitalMotion.ClassicElements()
        oe.a = earthGravBody.radEquator + 7500e3  # meters
        oe.e = 0.01
        oe.i = 30.0 * macros.D2R
        oe.Omega = 60.0 * macros.D2R
        oe.omega = 15.0 * macros.D2R
        oe.f = 90.0 * macros.D2R
        r_CN, rDot_CN = orbitalMotion.elem2rv(earthGravBody.mu, oe)

    elif orbit_config == 'no orbit':  # spacecraft taken out of orbit, ex. floating in deep space
        r_CN = np.array([1, 0, 0])  # [m]
        rDot_CN = np.array([1., 0., 0.])  # [m/s]

    else:
        raise Exception("orbit_config must be 'LEO' or 'no orbit'")

    # Set chaser spacecraft translational states relative to target
    if relpos_config == "alongtrackahead":
        rHat = rDot_CN / np.linalg.norm(rDot_CN)
    elif relpos_config == "alongtrackbehind":
        rHat = - rDot_CN / np.linalg.norm(rDot_CN)
    elif relpos_config == "radial":
        rHat = r_CN / np.linalg.norm(r_CN)
    elif relpos_config == "antiradial":
        rHat = - r_CN / np.linalg.norm(r_CN)
    else:
        raise Exception("relpos_config must be 'alongtrackahead', 'alongtrackbehind', 'radial', or 'antiradial'")

    scSim.r_P1B1_B1 = scSim.r_P1B1_mag * rHat
    scSim.r_P2B2_B2 = - scSim.r_P2B2_mag * rHat
    scSim.r_P2P1_B1Init = scSim.r_P2P1_mag * rHat
    r_B2B1 = scSim.r_P1B1_B1 + scSim.r_P2P1_B1Init - scSim.r_P2B2_B2

    # Center of mass calculation
    total_mass_1 = scSim.scObject1.hub.mHub
    total_mass_2 = scSim.scObject2.hub.mHub
    scSim.r_B1C = - (total_mass_1+ total_mass_2 * r_B2B1) / (total_mass_1 + total_mass_2)
    scSim.r_B2C = scSim.r_B1C + r_B2B1

    # Compute rotational states
    sigma_B1N = [[0.0], [0.0], [0.0]]
    sigma_B2N = [[0.0], [0.0], [0.0]]
    omega_CN = np.array([0.0, 0.0, 0.0])  # [rad/s]
    omega_B1N_B1 = omega_CN
    omega_B2N_B2 = omega_CN
    rDot_B1N_N = rDot_CN + np.cross(omega_CN, scSim.r_B1C)
    rDot_B2N_N = rDot_CN + np.cross(omega_CN, scSim.r_B2C)

    # Set the initial values for all spacecraft states
    scSim.scObject1.hub.r_CN_NInit = scSim.r_B1C + r_CN
    scSim.scObject1.hub.v_CN_NInit = rDot_B1N_N
    scSim.scObject1.hub.sigma_BNInit = sigma_B1N
    scSim.scObject1.hub.omega_BN_BInit = omega_B1N_B1
    scSim.scObject2.hub.r_CN_NInit = scSim.r_B2C + r_CN
    scSim.scObject2.hub.v_CN_NInit = rDot_B2N_N
    scSim.scObject2.hub.sigma_BNInit = sigma_B2N
    scSim.scObject2.hub.omega_BN_BInit = omega_B2N_B2


def set_up_constraint_effector(scSim, gain):
    # Set up the constraint effector
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.setR_P1B1_B1(scSim.r_P1B1_B1)
    constraintEffector.setR_P2B2_B2(scSim.r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init(scSim.r_P2P1_B1Init)
    constraintEffector.setAlpha(gain)
    constraintEffector.setBeta(constraintEffector.getAlpha())
    constraintEffector.ModelTag = "constraintEffector"

    # Add constraints to both spacecraft
    scSim.scObject1.addDynamicEffector(constraintEffector)
    scSim.scObject2.addDynamicEffector(constraintEffector)
    scSim.AddModelToTask(scSim.simTaskName, constraintEffector)


def set_up_maneuver(scSim, maneuver_config):
    # Create an external force/torque module
    scSim.extFT = extForceTorque.ExtForceTorque()

    # Set up an orbital maneuver translating the chaser
    if maneuver_config == "orbit":
        scSim.extFT.ModelTag = "appliedForce"
        scSim.extFT.extForce_B = [[0.0], [0.0], [0.0]]

    # Set up an attitude maneuver rotating the chaser
    elif maneuver_config == "attitude":
        scSim.extFT.ModelTag = "appliedTorque"

        # create the FSW vehicle configuration message
        vehicleConfigOut = messaging.VehicleConfigMsgPayload()
        vehicleConfigOut.ISCPntB_B = [element for row in scSim.scObject1.hub.IHubPntBc_B for element in row]
        scSim.vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

        # add the simple Navigation sensor module
        sNavObject = simpleNav.SimpleNav()
        sNavObject.ModelTag = "SimpleNavigation"
        scSim.AddModelToTask(scSim.simTaskName, sNavObject)
        sNavObject.scStateInMsg.subscribeTo(scSim.scObject1.scStateOutMsg)

        RefStateInData = messaging.AttRefMsgPayload()  # Create a structure for the input message
        sigma_R0N = np.array([0, 0, np.tan(np.pi / 4)])
        RefStateInData.sigma_RN = sigma_R0N
        omega_R0N_N = np.array([0.0, 0.0, 0.0])
        RefStateInData.omega_RN_N = omega_R0N_N
        scSim.attRefMsg = messaging.AttRefMsg().write(RefStateInData)

        # Set up the attitude tracking error evaluation module
        scSim.attError = attTrackingError.attTrackingError()
        scSim.attError.ModelTag = "scSim.attErrorHillPoint"
        scSim.AddModelToTask(scSim.simTaskName, scSim.attError)
        scSim.attError.attRefInMsg.subscribeTo(scSim.attRefMsg)#rotationRef.attRefOutMsg)#attGuidance.attRefOutMsg)
        scSim.attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

        # setup the MRP Feedback control module
        scSim.mrpControl = mrpFeedback.mrpFeedback()
        scSim.mrpControl.ModelTag = "mrpFeedback"
        scSim.AddModelToTask(scSim.simTaskName, scSim.mrpControl)
        scSim.mrpControl.K = 1.0
        scSim.mrpControl.Ki = -1.0  # make value negative to turn off integral feedback
        scSim.mrpControl.P = 50.0
        scSim.mrpControl.integralLimit = 2. / scSim.mrpControl.Ki * 0.1
        scSim.mrpControl.guidInMsg.subscribeTo(scSim.attError.attGuidOutMsg)
        scSim.mrpControl.vehConfigInMsg.subscribeTo(scSim.vcMsg)

        # Log control torque commands
        scSim.torqueData = scSim.mrpControl.cmdTorqueOutMsg.recorder()
        scSim.AddModelToTask(scSim.simTaskName, scSim.torqueData)

    else:
        raise Exception("maneuver_config must be 'orbit' or 'attitude'")

    scSim.scObject1.addDynamicEffector(scSim.extFT)
    scSim.AddModelToTask(scSim.simTaskName, scSim.extFT)


def log_data(scSim):
    # Log the spacecraft state message
    scSim.datLog1 = scSim.scObject1.scStateOutMsg.recorder()
    scSim.datLog2 = scSim.scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog2)


def set_up_vizard(scSim):
    return vizSupport.enableUnityVisualization(scSim, scSim.simTaskName,
                                               [scSim.scObject1, scSim.scObject2],
                                               # saveFile=__file__
                                               )


def run_simulation(scSim, maneuver_config):
    t = time.time()

    # Initialize the simulation
    scSim.SetProgressBar(True)
    scSim.InitializeSimulation()

    scSim.ConfigureStopTime(macros.min2nano(5))
    scSim.ExecuteSimulation()

    if maneuver_config == "orbit":
        # low thrust aligned to push target
        dir = scSim.r_P2P1_B1Init / np.linalg.norm(scSim.r_P2P1_B1Init)
        scSim.extFT.extForce_B = -1.0*dir
    elif maneuver_config == "attitude":
        # connect command torque message
        scSim.extFT.cmdTorqueInMsg.subscribeTo(scSim.mrpControl.cmdTorqueOutMsg)

    scSim.ConfigureStopTime(macros.min2nano(10))
    scSim.ExecuteSimulation()

    # turn off the low thrust
    scSim.extFT.extForce_B = [[0.0], [0.0], [0.0]]

    # Run the simulation
    scSim.ConfigureStopTime(macros.min2nano(20))
    scSim.ExecuteSimulation()

    # Print Runtime
    scSim.runtime = time.time() - t
    print('Elapsed Time = ' + str(scSim.runtime) + ' seconds')


def process_data(scSim):
    # Grab the time vector
    scSim.time_data = scSim.datLog1.times() * macros.NANO2SEC
    num_steps = len(scSim.time_data)

    # Collect the logged spacecraft states
    r_B1N_N_hist = scSim.datLog1.r_BN_N
    scSim.sigma_B1N_hist = scSim.datLog1.sigma_BN
    r_B2N_N_hist = scSim.datLog2.r_BN_N
    scSim.sigma_B2N_hist = scSim.datLog2.sigma_BN

    # Compute constraint violations
    r_B1N_B1 = np.empty(r_B1N_N_hist.shape)
    r_B2N_B1 = np.empty(r_B2N_N_hist.shape)
    r_P2B2_B1 = np.empty(r_B1N_N_hist.shape)
    scSim.sigma_B2B1 = np.empty(scSim.sigma_B1N_hist.shape)
    for i in range(num_steps):
        dcm_B1N = RigidBodyKinematics.MRP2C(scSim.sigma_B1N_hist[i, :])
        r_B1N_B1[i, :] = dcm_B1N @ r_B1N_N_hist[i, :]
        r_B2N_B1[i, :] = dcm_B1N @ r_B2N_N_hist[i, :]
        dcm_NB2 = RigidBodyKinematics.MRP2C(scSim.sigma_B2N_hist[i, :]).transpose()
        r_P2B2_B1[i, :] = dcm_B1N @ dcm_NB2 @ scSim.r_P2B2_B2
        scSim.sigma_B2B1[i, :] = RigidBodyKinematics.subMRP(scSim.sigma_B2N_hist[i, :], scSim.sigma_B1N_hist[i, :])
    scSim.psi_B1 = r_B1N_B1 + scSim.r_P1B1_B1 + scSim.r_P2P1_B1Init - (r_B2N_B1 + r_P2B2_B1)


def plotting(scSim_list, gain_list, maneuver_config, show_plots):
    larger_size = 20
    smaller_size = 18
    fontdict = {'family': 'serif',
                'weight': 'normal',
                'size': larger_size}
    plt.rc('font', **fontdict)
    plt.rc('axes', labelsize=larger_size)
    plt.rc('xtick', labelsize=smaller_size)
    plt.rc('ytick', labelsize=smaller_size)
    plt.rc('legend', fontsize=smaller_size)
    plt.rcParams['figure.figsize'] = (7, 6)
    plt.close("all")

    # Collect plotting utilities
    figureList = {}
    n = len(gain_list) # number of data sets
    timeData = scSim_list[0].time_data/60 # Convert time data to minutes

    labels = []
    for i in range(n):
        labels.append(r'$\alpha = $' f"{gain_list[i]:.0e}")

    # Plot direction constraint violations
    plt.figure()
    for i in range(n):
        plt.semilogy(timeData, np.linalg.norm(scSim_list[i].psi_B1, axis=1))
        plt.xlabel('time [min]')
        plt.ylabel('Direction Constraint Violation ' r'$\psi$ [m]')
        plt.axis("tight")
    plt.legend(labels, loc='lower right')
    pltName = fileName + maneuver_config + "maneuver_" + "DirectionConstraint"
    figureList[pltName] = plt.gcf()

    # Plot attitude constraint violations
    plt.figure()
    for i in range(n):
        plt.semilogy(timeData, np.linalg.norm(4 * np.arctan(scSim_list[i].sigma_B2B1) * macros.R2D, axis=1))
        plt.xlabel('time [min]')
        plt.ylabel('Attitude Constraint Violation ' r'$\phi$ [deg]')
        plt.axis("tight")
    plt.legend(labels, loc='lower right')
    pltName = fileName + maneuver_config + "maneuver_" + "AttitudeConstraint"
    figureList[pltName] = plt.gcf()

    # Plot gain performance
    fig, ax1 = plt.subplots()
    trans_cnst = []
    rot_cnst = []
    runtimes = []
    for i in range(n):
        runtimes.append(scSim_list[i].runtime)
        trans_cnst.append(np.mean(np.linalg.norm(scSim_list[i].psi_B1, axis=1)))
        rot_cnst.append(np.mean(np.linalg.norm(4 * np.arctan(scSim_list[i].sigma_B2B1) * macros.R2D, axis=1)))

    cnst_color = 'tab:blue'
    ax1.loglog(gain_list,trans_cnst, color=cnst_color, linestyle='-')
    ax1.loglog(gain_list, rot_cnst, color=cnst_color, linestyle='--')
    ax1.set_xlabel('gain')
    ax1.set_ylabel('average constraint violation [m, deg]', color=cnst_color)
    ax1.tick_params(axis='y', labelcolor=cnst_color)
    ax1.legend(['direction', 'attitude'], loc='upper center')

    runtime_color = 'tab:orange'
    ax2 = ax1.twinx()
    ax2.loglog(gain_list, runtimes, color=runtime_color)
    ax2.set_ylabel('runtime [sec]', color=runtime_color)
    ax2.tick_params(axis='y', labelcolor=runtime_color)

    pltName = fileName + maneuver_config + "maneuver_" + "GainPerformance"
    figureList[pltName] = plt.gcf()

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList

if __name__ == "__main__":
    run(
        True,  # show_plots
        np.logspace(0, 4, 5),  # gain_list
        relpos_config="alongtrackahead",  # relative position ["alongtrackahead", "alongtrackbehind", "radial", or "antiradial"]
        orbit_config="LEO",  # in orbit or freefloating ["LEO", "no orbit"]
        maneuver_config="attitude",  # maneuver type ["orbit", "attitude"]
        sc_model='MEV2',  # spacecraft model ['bskSat', 'MEV1', 'MEV2']
    )
