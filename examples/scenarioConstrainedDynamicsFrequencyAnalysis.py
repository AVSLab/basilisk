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
impact of different gains on the motion of two spacecraft with solar panels. Specifically, it looks at the coupling
between the natural frequency of the solar arrays and the fictitious frequency that the constraint effector introduces.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioConstrainedDynamicsFrequencyAnalysis.py

The scenario outputs two plots. The first one shows the solar array angle errors for different gains to show how they
change for different arm stiffness values. The second one shows the results of an FFT on the spacecraft's attitude,
which highlights the frequencies of the solar arrays and the fictitious frequency introduced for each gain.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, gain_list = [1E1, 1E2, 1E3, 1E4, 1E5], sc_model = 'MEV2'

The time history for the solar array angle errors between the approximate and truth models is shown below for different
values of the gains.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsFrequencyAnalysisPanelAngleErrors.svg
   :align: center

The figure below shows the FFT of the attitude error between both spacecraft. Each gain curve has two peaks: one at the
solar panel's natural frequency, and another at a gain-specific frequency, which is fictitious. Note how this frequency
is higher for higher gains, as the corresponding dynamics are stiffer. For lower gains, the fictitious frequency
resonates with the panel's natural frequency, which leads to large errors between the approximate and truth model since
the fictitious frequency does not appear in the truth model.

.. image:: /_images/Scenarios/scenarioConstrainedDynamicsFrequencyAnalysisFFT.svg
   :align: center

"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Illustrates coupled spacecraft frequency analysis with components.
#   Author:             João Vaz Carneiro
#   Creation Date:      Oct 2, 2025
#

# Basilisk imports
from Basilisk.utilities import (SimulationBaseClass, macros, RigidBodyKinematics, vizSupport, unitTestSupport)
from Basilisk.simulation import (spacecraft, constraintDynamicEffector, svIntegrators, hingedRigidBodyStateEffector)
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
    def __init__(self, dynRate=0.001):
        self.dynRateSec = dynRate
        self.dynRateNanos = macros.sec2nano(dynRate)
        SimulationBaseClass.SimBaseClass.__init__(self)

        # Create simulation variable names
        self.simTaskName = "simTask"
        simProcessName = "simProcess"

        # Create the dynamics task and set the integration update time
        self.dynProcess = self.CreateNewProcess(simProcessName)
        self.dynProcess.addTask(self.CreateNewTask(self.simTaskName, self.dynRateNanos))


class solar_panel_geometry:
    mass = 100.0
    IPntS_S = np.array([[150.0, 0.0, 0.0], [0.0, 300.0, 0.0], [0.0, 0.0, 150.0]])
    d = 1.0  # note this is the distance from hinge point to hinged rigid body COM, not the damping coefficient
    r_H1B_B = np.array([0.0, 1.0, 1.0])
    r_H2B_B = np.array([0.0, -1.0, -1.0])
    dcm_H1B = np.array([[0.0, -1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])
    dcm_H2B = np.array([[0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])

    xi = 0.2
    freq = 2 * 2 * np.pi
    k = IPntS_S[1, 1] * freq ** 2
    c = IPntS_S[1, 1] * 2 * xi * freq
    thetaInit = 0.0 * macros.D2R
    thetaDotInit = 0.0 * macros.D2R


def plot_panel_angles_error(timeData, sp1ThetaLog, sp3ThetaLog, gain_list):
    plt.figure(1)
    for idx, gain in enumerate(gain_list):
        plt.semilogy(timeData[idx], abs((sp1ThetaLog[idx] - sp3ThetaLog[idx])) * macros.R2D,
                     color=unitTestSupport.getLineColor(idx, len(gain_list)), label=r'$\alpha=$' f'{gain:.0e}')
    plt.legend(loc='best')
    plt.xlabel('Time [sec]')
    plt.ylabel('Angle Error ' r'$\Delta \theta$ [deg]')


def plot_fft_panels(sampling_rate, sigma_B2B1_list, gain_list):
    X = []
    f = None
    T = None
    for sigma_B2B1 in sigma_B2B1_list:
        data = np.abs(np.fft.fft(sigma_B2B1[:, 0]))

        N = len(data)
        n = np.arange(N)
        T = N / sampling_rate
        freq = n / T

        # Get the one-sided spectrum
        n_oneside = N // 2
        f = freq[:n_oneside]
        X.append(data[:n_oneside] / n_oneside)

    plt.figure(2)
    for idx, gain in enumerate(gain_list):
        plt.loglog(f, X[idx], label=r'$\alpha=$' f'{gain:.0e}', color=unitTestSupport.getLineColor(idx, len(gain_list)))
        print(f'Frequency of highest amplitude for gain={gain} is {np.unravel_index(X[idx].argmax(), N // 2)[0] / T} Hz.')
    plt.legend(loc='best')
    plt.xlabel('Freq [Hz]')
    plt.ylabel(r'$|X|$')
    plt.grid(which='both')


def run(show_plots, gain_list, sc_model):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
        gain_list (float): Choose which gain values to test
        sc_model (str): Choose which spacecraft models to use between ``bskSat``, ``MEV1``, or ``MEV2``
    """
    scSim_list = []
    for gain in gain_list:
        scSim = SimBaseClass()
        create_spacecraft(scSim, sc_model)
        define_initial_conditions(scSim,)
        set_up_constraint_effector(scSim, gain)
        set_up_additional_effectors(scSim)
        log_data(scSim)

        set_up_vizard(scSim)
        run_simulation(scSim)
        process_data(scSim)

        scSim_list.append(scSim)

    return plotting(scSim_list, gain_list, show_plots)


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
        scSim.r_P2P1_mag = 0.1  # [m] arm length |r_P2P1|

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
        scSim.r_P2P1_mag = 0.5  # [m] arm length |r_P2P1|

    elif sc_model == 'MEV2':  # chaser is MEV-2 and target is Intelsat 10-02
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
        scSim.r_P2P1_mag = 0.5  # [m] arm length |r_P2P1|

    else:
        raise Exception("sc_model must be 'bskSat', 'MEV1', or 'MEV2'")

    # Create single spacecraft truth model for comparison
    scSim.scObjectT = spacecraft.Spacecraft()
    scSim.scObjectT.ModelTag = "spacecraftBodyT"

    # Set truth spacecraft integrator to be the same as the constraint effector model
    integratorObjectT = svIntegrators.svIntegratorRKF45(scSim.scObjectT)
    scSim.scObjectT.setIntegrator(integratorObjectT)
    integratorObjectT.this.disown()

    scSim.AddModelToTask(scSim.simTaskName, scSim.scObjectT)


def define_initial_conditions(scSim):
    # Note: here we assume that the N, B1 and B2 frames are identical, therefore no frame is defined
    r_CN = np.array([1, 0, 0])  # [m]
    rDot_CN = np.array([0., 0., 0.])  # [m/s]
    rHat = r_CN / np.linalg.norm(r_CN)

    scSim.r_P1B1_B1 = scSim.r_P1B1_mag * rHat
    scSim.r_P2B2_B2 = - scSim.r_P2B2_mag * rHat
    scSim.r_P2P1_B1Init = scSim.r_P2P1_mag * rHat
    r_B2B1 = scSim.r_P1B1_B1 + scSim.r_P2P1_B1Init - scSim.r_P2B2_B2

    # Center of mass calculation
    r_C1B1 = np.zeros(3)
    r_C2B2 = np.zeros(3)

    total_mass_1 = scSim.scObject1.hub.mHub
    total_mass_2 = scSim.scObject2.hub.mHub
    r_C1B1 = (total_mass_1 * r_C1B1 + solar_panel_geometry.mass * (solar_panel_geometry.r_H1B_B - solar_panel_geometry.dcm_H1B[0, :] * solar_panel_geometry.d)) / (total_mass_1 + solar_panel_geometry.mass)
    r_C2B2 = (total_mass_1 * r_C2B2 + solar_panel_geometry.mass * (solar_panel_geometry.r_H2B_B - solar_panel_geometry.dcm_H2B[0, :] * solar_panel_geometry.d)) / (total_mass_2 + solar_panel_geometry.mass)

    total_mass_1 += solar_panel_geometry.mass
    total_mass_2 += solar_panel_geometry.mass
    scSim.r_B1C = - (total_mass_1 * r_C1B1 + total_mass_2 * (r_C2B2 + r_B2B1)) / (total_mass_1 + total_mass_2)
    scSim.r_B2C = scSim.r_B1C + r_B2B1

    # Compute rotational states
    sigma_B1N = [[0.0], [0.0], [0.0]]
    sigma_B2N = [[0.0], [0.0], [0.0]]
    omega_CN = np.array([0.0, 0.0, 0.0])
    omega_B1N_B1 = omega_CN
    omega_B2N_B2 = omega_CN
    rDot_B1N_N = rDot_CN + np.cross(omega_CN, scSim.r_B1C)
    rDot_B2N_N = rDot_CN + np.cross(omega_CN, scSim.r_B2C)

    # Set the initial values for all spacecraft states
    scSim.scObject1.hub.r_CN_NInit = r_C1B1 + scSim.r_B1C + r_CN
    scSim.scObject1.hub.v_CN_NInit = rDot_B1N_N
    scSim.scObject1.hub.sigma_BNInit = sigma_B1N
    scSim.scObject1.hub.omega_BN_BInit = omega_B1N_B1
    scSim.scObject2.hub.r_CN_NInit = r_C2B2 + scSim.r_B2C + r_CN
    scSim.scObject2.hub.v_CN_NInit = rDot_B2N_N
    scSim.scObject2.hub.sigma_BNInit = sigma_B2N
    scSim.scObject2.hub.omega_BN_BInit = omega_B2N_B2

    # Create single spacecraft truth model for comparison
    scSim.r_BcC = (scSim.scObject1.hub.mHub * scSim.r_B1C + scSim.scObject2.hub.mHub * scSim.r_B2C) / (scSim.scObject1.hub.mHub + scSim.scObject2.hub.mHub)
    rTilde_B1Bc = np.array(RigidBodyKinematics.v3Tilde(scSim.r_B1C - scSim.r_BcC))
    rTilde_B2Bc = np.array(RigidBodyKinematics.v3Tilde(scSim.r_B2C - scSim.r_BcC))
    IPntBc = (scSim.scObject1.hub.IHubPntBc_B - scSim.scObject1.hub.mHub * rTilde_B1Bc @ rTilde_B1Bc + scSim.scObject2.hub.IHubPntBc_B - scSim.scObject2.hub.mHub * rTilde_B2Bc @ rTilde_B2Bc)

    scSim.scObjectT.hub.IHubPntBc_B = IPntBc
    scSim.scObjectT.hub.mHub = scSim.scObject1.hub.mHub + scSim.scObject2.hub.mHub

    # Set initial state values
    scSim.scObjectT.hub.r_CN_NInit = r_CN
    scSim.scObjectT.hub.v_CN_NInit = rDot_CN
    scSim.scObjectT.hub.sigma_BNInit = sigma_B1N
    scSim.scObjectT.hub.omega_BN_BInit = omega_CN


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


def set_up_additional_effectors(scSim):
    # Set up solar panel(s)
    scSim.solarPanel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    scSim.solarPanel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # define spacecraft 1's solar panel properties
    scSim.solarPanel1.mass = solar_panel_geometry.mass
    scSim.solarPanel1.IPntS_S = solar_panel_geometry.IPntS_S
    scSim.solarPanel1.d = solar_panel_geometry.d
    scSim.solarPanel1.k = solar_panel_geometry.k
    scSim.solarPanel1.c = solar_panel_geometry.c
    scSim.solarPanel1.r_HB_B = solar_panel_geometry.r_H1B_B
    scSim.solarPanel1.dcm_HB = solar_panel_geometry.dcm_H1B
    scSim.solarPanel1.thetaInit = solar_panel_geometry.thetaInit
    scSim.solarPanel1.thetaDotInit = solar_panel_geometry.thetaDotInit

    # define spacecraft 2's solar panel properties
    scSim.solarPanel2.mass = solar_panel_geometry.mass
    scSim.solarPanel2.IPntS_S = solar_panel_geometry.IPntS_S
    scSim.solarPanel2.d = solar_panel_geometry.d
    scSim.solarPanel2.k = solar_panel_geometry.k
    scSim.solarPanel2.c = solar_panel_geometry.c
    scSim.solarPanel2.r_HB_B = solar_panel_geometry.r_H2B_B
    scSim.solarPanel2.dcm_HB = solar_panel_geometry.dcm_H2B
    scSim.solarPanel2.thetaInit = solar_panel_geometry.thetaInit
    scSim.solarPanel2.thetaDotInit = solar_panel_geometry.thetaDotInit

    # add solar panels to spacecraft
    scSim.scObject1.addStateEffector(scSim.solarPanel1)
    scSim.scObject2.addStateEffector(scSim.solarPanel2)

    scSim.solarPanel3 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    scSim.solarPanel4 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # define the truth spacecraft's first solar panel properties
    scSim.solarPanel3.mass = solar_panel_geometry.mass
    scSim.solarPanel3.IPntS_S = solar_panel_geometry.IPntS_S
    scSim.solarPanel3.d = solar_panel_geometry.d
    scSim.solarPanel3.k = solar_panel_geometry.k
    scSim.solarPanel3.c = solar_panel_geometry.c
    scSim.solarPanel3.r_HB_B = solar_panel_geometry.r_H1B_B + scSim.r_B1C - scSim.r_BcC
    scSim.solarPanel3.dcm_HB = solar_panel_geometry.dcm_H1B
    scSim.solarPanel3.thetaInit = solar_panel_geometry.thetaInit
    scSim.solarPanel3.thetaDotInit = solar_panel_geometry.thetaDotInit

    # define the truth spacecraft's second solar panel properties
    scSim.solarPanel4.mass = solar_panel_geometry.mass
    scSim.solarPanel4.IPntS_S = solar_panel_geometry.IPntS_S
    scSim.solarPanel4.d = solar_panel_geometry.d
    scSim.solarPanel4.k = solar_panel_geometry.k
    scSim.solarPanel4.c = solar_panel_geometry.c
    scSim.solarPanel4.r_HB_B = solar_panel_geometry.r_H2B_B + scSim.r_B2C - scSim.r_BcC
    scSim.solarPanel4.dcm_HB = solar_panel_geometry.dcm_H2B
    scSim.solarPanel4.thetaInit = solar_panel_geometry.thetaInit
    scSim.solarPanel4.thetaDotInit = solar_panel_geometry.thetaDotInit

    # add solar panels to spacecraft
    scSim.scObjectT.addStateEffector(scSim.solarPanel3)
    scSim.scObjectT.addStateEffector(scSim.solarPanel4)


def log_data(scSim):
    # Log the spacecraft state message
    scSim.datLog1 = scSim.scObject1.scStateOutMsg.recorder()
    scSim.datLog2 = scSim.scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog2)

    scSim.sp1Log = scSim.solarPanel1.hingedRigidBodyOutMsg.recorder()
    scSim.sp2Log = scSim.solarPanel2.hingedRigidBodyOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.sp1Log)
    scSim.AddModelToTask(scSim.simTaskName, scSim.sp2Log)

    scSim.sp3Log = scSim.solarPanel3.hingedRigidBodyOutMsg.recorder()
    scSim.sp4Log = scSim.solarPanel4.hingedRigidBodyOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.sp3Log)
    scSim.AddModelToTask(scSim.simTaskName, scSim.sp4Log)

    scSim.datLogT = scSim.scObjectT.scStateOutMsg.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLogT)


def set_up_vizard(scSim):
    return vizSupport.enableUnityVisualization(scSim, scSim.simTaskName,
                                               [scSim.scObject1, scSim.scObject2, scSim.scObjectT],
                                               # saveFile=__file__
                                               )


def run_simulation(scSim):
    t = time.time()

    # Initialize the simulation
    scSim.SetProgressBar(True)
    scSim.InitializeSimulation()

    # Add simulation perturbations
    scSim.scObject1.dynManager.getStateObject(scSim.solarPanel1.nameOfThetaDotState).setState([[7 * macros.D2R]])
    scSim.scObjectT.dynManager.getStateObject(scSim.solarPanel3.nameOfThetaDotState).setState([[7 * macros.D2R]])

    # Run the simulation
    scSim.ConfigureStopTime(macros.sec2nano(10))
    scSim.ExecuteSimulation()

    # Print Runtime
    print('Elapsed Time = ' + str(time.time() - t) + ' seconds')


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


def plotting(scSim_list, gain_list, show_plots):
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
    plt.rcParams["figure.figsize"] = (7, 6)
    plt.rcParams['axes.autolimit_mode'] = 'data'  # no “round number” padding
    plt.rcParams['axes.xmargin'] = 0
    plt.close("all")

    spTimes = [scSim.sp1Log.times() * macros.NANO2SEC for scSim in scSim_list]
    sp1ThetaLog = [scSim.sp1Log.theta for scSim in scSim_list]
    sp3ThetaLog = [scSim.sp3Log.theta for scSim in scSim_list]
    plot_panel_angles_error(spTimes, sp1ThetaLog, sp3ThetaLog, gain_list)
    figureList = {}
    pltName = fileName + "PanelAngleErrors"
    figureList[pltName] = plt.figure(1)

    # Plot FFT
    plot_fft_panels(1 / scSim_list[0].dynRateSec, [scSim.sigma_B2B1 for scSim in scSim_list], gain_list)
    pltName = fileName + "FFT"
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(
        True,  # show_plots
        [1E1, 1E2, 1E3, 1E4, 1E5],  # gain_list
        sc_model='MEV2',  # spacecraft model ['bskSat', 'testing', 'MEV1', 'MEV2']
    )
