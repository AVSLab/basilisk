import os
import time
import numpy as np
from scipy.stats import chi2

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.utilities import (SimulationBaseClass, macros, fswSetupRW,
                                    simIncludeGravBody,simIncludeRW, unitTestSupport)
from Basilisk.fswAlgorithms import (attTrackingError, mrpFeedback, inertial3D, rwMotorTorque)
from Basilisk.simulation import (spacecraft, simpleNav, reactionWheelStateEffector)

from src.parameters import SimConfig, NoiseParameters
from src.ukf import UKFSetup

class FID:
    def __init__(self, sim_config, fid_config, true_mode, noise_parameters):
        self.sim_config = sim_config
        self.fid_config = fid_config
        self.true_mode = true_mode
        self.noise_parameters = noise_parameters

        # Derived parameters
        self.k_stop = int(25 + self.fid_config['moving_window'] * 2.0)

    def setup_basilisk_simulation(simTime=20.0):
        """
        Sets up the basic Basilisk simulation framework with a spacecraft object.

        Args:
            simTime (float): Duration of simulation in minutes

        Returns:
            tuple: (scSim, scObject, simTaskName, simulationTime, simulationTimeStep)
        """
        # Simulation identifiers
        simTaskName = "simTask"
        simProcessName = "simProcess"

        # Create a SimBaseClass instance
        scSim = SimulationBaseClass.SimBaseClass()

        # Convert simTime from minutes to nanoseconds
        simulationTime = macros.min2nano(simTime)

        # Create a process
        dynProcess = scSim.CreateNewProcess(simProcessName)

        # Integration time step
        simulationTimeStep = macros.sec2nano(0.1)
        dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

        # Create and configure spacecraft object
        scObject = spacecraft.Spacecraft()
        scObject.ModelTag = "bsk-Sat"

         # Define the simulation inertia
        I = [10.0, 0., 0.,
             0., 5.0, 0.,
             0., 0., 7.5]
        scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

        return scSim, scObject, simTaskName, simulationTime, simulationTimeStep


    def setup_simulation(scSim, simTaskName, scObject):
        # Add spacecraft object to the simulation process with priority 1
        scSim.AddModelToTask(simTaskName, scObject, 1)

        # Clear prior gravitational bodies and setup a new gravity factory
        gravFactory = simIncludeGravBody.gravBodyFactory()

        # Setup Mars Gravity Body
        mars = gravFactory.createMars()
        mars.isCentralBody = True  # set Mars as central gravitational body
        mu = mars.mu

        # Attach gravity model to spacecraft
        gravFactory.addBodiesTo(scObject)


    def setup_reaction_wheels(scSim, simTaskName, scObject, useJitterSimple=False):
        """
        Setup reaction wheels and add them to the spacecraft and simulation.

        Args:
            scSim (SimulationBaseClass.SimBaseClass): The Basilisk simulation object
            simTaskName (str): Name of the simulation task
            scObject (spacecraft.Spacecraft): The spacecraft object
            useJitterSimple (bool): If True, use jitter model for RW; else balanced wheels

        Returns:
            tuple: (rwFactory, rwStateEffector, numRW)
        """
        rwFactory = simIncludeRW.rwFactory()

        varRWModel = messaging.BalancedWheels
        if useJitterSimple:
            varRWModel = messaging.JitterSimple

        # Create wheels
        RW1 = rwFactory.create('Honeywell_HR16', [1, 0, 0], maxMomentum=50., Omega=100., RWModel=varRWModel)
        RW2 = rwFactory.create('Honeywell_HR16', [0, 1, 0], maxMomentum=50., Omega=200., RWModel=varRWModel)
        RW3 = rwFactory.create('Honeywell_HR16', [0, 0, 1], maxMomentum=50., Omega=300., rWB_B=[0.5, 0.5, 0.5], RWModel=varRWModel)

        numRW = rwFactory.getNumOfDevices()

        # Create RW state effector container and add to spacecraft
        rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        rwStateEffector.ModelTag = "RW_cluster"
        rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

        # Add RW state effector to the sim task
        scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

        return rwFactory, rwStateEffector, numRW

    def setup_guidance_navigation(scSim, simTaskName):
        """
        Setup the navigation, guidance, and attitude error modules.

        Args:
            scSim (SimulationBaseClass.SimBaseClass): The simulation object
            simTaskName (str): The simulation task name

        Returns:
            tuple: (sNavObject, inertial3DObj, attError)
        """
        # Simple Navigation sensor module
        sNavObject = simpleNav.SimpleNav()
        sNavObject.ModelTag = "SimpleNavigation"
        scSim.AddModelToTask(simTaskName, sNavObject)

        # Inertial 3D guidance module
        inertial3DObj = inertial3D.inertial3D()
        inertial3DObj.ModelTag = "inertial3D"
        scSim.AddModelToTask(simTaskName, inertial3DObj)
        inertial3DObj.sigma_R0N = [0., 0., 0.]  # desired inertial orientation

        # Attitude tracking error evaluation module
        attError = attTrackingError.attTrackingError()
        attError.ModelTag = "attErrorInertial3D"
        scSim.AddModelToTask(simTaskName, attError)

        return sNavObject, attError


    @staticmethod
    def setup_mrp_control(scSim, simTaskName, sim_config: SimConfig):
        mrpControl = mrpFeedback.mrpFeedback()
        mrpControl.ModelTag = "mrpFeedback"
        scSim.AddModelToTask(simTaskName, mrpControl)

        # Access attributes directly from SimConfig instance
        mrpControl.K = getattr(sim_config, 'controller_K', 3.5)
        mrpControl.Ki = getattr(sim_config, 'controller_Ki', -1)  # negative disables integral feedback
        mrpControl.P = getattr(sim_config, 'controller_P', 30.0)

        if mrpControl.Ki > 0:
            mrpControl.integralLimit = 2.0 / mrpControl.Ki * 0.1
        else:
            mrpControl.integralLimit = 0.0

        return mrpControl

    def setup_rw_motor_torque(scSim, simTaskName):
        # Add module that maps the Lr control torque into the RW motor torques
        rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
        rwMotorTorqueObj.ModelTag = "rwMotorTorque"
        scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

        # Make the RW control all three body axes
        controlAxes_B = [1, 0, 0,
                        0, 1, 0,
                        0, 0, 1]
        rwMotorTorqueObj.controlAxes_B = controlAxes_B

        return rwMotorTorqueObj
    

    def setup_logging_and_rw_config(simTaskName, samplingTime, scSim, rwFactory, rwMotorTorqueObj, attError, sNavObject):
        # Setup loggers
        rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
        attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
        snTransLog = sNavObject.transOutMsg.recorder(samplingTime)

        # Add loggers to simulation task
        scSim.AddModelToTask(simTaskName, rwMotorLog)
        scSim.AddModelToTask(simTaskName, attErrorLog)
        scSim.AddModelToTask(simTaskName, snTransLog)

        # Setup RW FSW config message
        fswSetupRW.clearSetup()
        for key, rw in rwFactory.rwList.items():
            fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
        fswRwParamMsg = fswSetupRW.writeConfigMessage()

        # Return logs and config message so caller can keep references
        return {
            "rwMotorLog": rwMotorLog,
            "attErrorLog": attErrorLog,
            "snTransLog": snTransLog
        }, fswRwParamMsg


    @staticmethod
    def run_fault_trials(N_vals, moving_window, sim_config=None):
        trial_configs = []

        for N in N_vals:
            n_trials = 10

            for trial in range(n_trials):
                np.random.seed(trial)

                sim_config = SimConfig(moving_window=moving_window)
                sim_config.N_trials = 1
                sim_config.save_data = False
                sim_config.rlevel = 'hig'
                sim_config.inc_time = 1
                sim_config.start_time = np.random.randint(0, 7102)
                sim_config.end_time = sim_config.start_time + 1000

                fid_config = {
                    'action_flag': True,
                    'moving_window': moving_window,
                    'N_hypo': 4,
                    'crit': 0.95,
                    'alpha': 0.05
                }

                true_mode = np.random.randint(-1, 4)

                trial_configs.append((sim_config, fid_config, true_mode))

        return trial_configs
    

    def active_fault_ID(scSim, simTaskName, numRW, sNavObject, attError, sim_config, mrpControl, 
                        rwMotorTorqueObj, logs, attEstimator, attEstimatorLog, sweep_window):
        

        # Initial printing statements
        print("Running active fault ID with inputs:")
        print(f"Simulation Task: {simTaskName}")
        print(f"Planetary Body:" " Mars")
        print(f"Number of Reaction Wheels: {numRW}")
        print(f"Sweep Window: {sweep_window}")

        # Determine simulation time and stopping point for FID
        start_time = 0
        inc_time = 1
        k_stop = int(25 + sweep_window * 2.0)
        simulationTime = np.arange(start_time, start_time + (k_stop + 1) * inc_time, inc_time)

        # Simualte activate fault ID
        x_hist = [attEstimator.stateInit]
        y_hist = []
        u_hist = []

        for k in range(len(simulationTime) - 1):
            t = simulationTime[k + 1]

            scSim.ConfigureStopTime(t)
            scSim.ExecuteSimulation()

            trans_msg = sNavObject.transOutMsg.read()
            att_msg = sNavObject.attOutMsg.read()

            r_BN_N = trans_msg.r_BN_N
            v_BN_N = trans_msg.v_BN_N

            sigma_BN = att_msg.sigma_BN
            omega_BN_B = att_msg.omega_BN_B

            x_next = np.hstack([r_BN_N, v_BN_N, sigma_BN, omega_BN_B])
            x_hist.append(x_next)

            # Controller gains (tune or get from sim_config)
            K = getattr(sim_config, 'controller_K', 0.01)
            P = getattr(sim_config, 'controller_P', 0.1)

            noise_params = NoiseParameters()
            v = np.random.multivariate_normal(np.zeros(noise_params.R.shape[0]), noise_params.R)
            y_next = noise_params.H @ x_next[6:] + v
            y_hist.append(y_next)

            # Control law (PD control)
            sigma_BR = att_msg.sigma_BN
            sigma_BR = np.array(sigma_BR)
            omega_BN_B = np.array(omega_BN_B)
            torque_msg = mrpControl.cmdTorqueOutMsg.read()
            u_apply = np.array(torque_msg.torqueRequestBody)        
            u_apply = np.array(u_apply)
            u_hist.append(u_apply)

            exit()
            # Store filtering data
            mu_H_pos_hist.append(mu_H_pos.copy())
            cov_H_pos_hist.append(cov_H_pos.copy())
            inno_H_pri_hist.append(inno_H_pri.copy())
            S_H_pri_hist.append(S_H_pri.copy())

            if k >= sweep_window:
                for m in range(n_Hypo):
                    # Stack innovations over moving window
                    inno_used = np.vstack([
                        inno_H_pri_hist[-i - 1][:, m].reshape(-1, 1)
                        for i in reversed(range(fid_config['moving_window']))
                    ])

                    # Stack block-diagonal innovation covariance matrices
                    tmp = [
                        S_H_pri_hist[-i - 1][:, :, m]
                        for i in reversed(range(fid_config['moving_window']))
                    ]

                    S_used = np.block([
                        [tmp[i] if i == j else np.zeros_like(tmp[0])
                        for j in range(fid_config['moving_window'])]
                        for i in range(fid_config['moving_window'])
                    ])

                    try:
                        chi = float(inno_used.T @ np.linalg.inv(S_used) @ inno_used) / fid_config['moving_window']
                    except np.linalg.LinAlgError:
                        chi = np.inf

                    inno_whiten = inno_used.reshape(ny, fid_config['moving_window'])

                    for jj in range(fid_config['moving_window']):
                        inno_jj = inno_whiten[:, jj]
                        S_jj = S_used[ny * jj : ny * (jj + 1), ny * jj : ny * (jj + 1)]
                        try:
                            L = np.linalg.cholesky(S_jj)
                            inno_whiten_jj = np.linalg.solve(L, inno_jj)
                            inno_whiten[:, jj] = inno_whiten_jj
                        except np.linalg.LinAlgError:
                            inno_whiten[:, jj] = np.full_like(inno_jj, np.inf)

                    chi_sr = np.sum(inno_whiten**2) / fid_config['moving_window']

                    inno_used_whiten = inno_whiten.reshape(ny * fid_config['moving_window'], 1)
                    S_used_whiten = np.eye(ny * fid_config['moving_window'])

                    if chi_crit_lo <= chi_sr <= chi_crit_up:
                        logdet = log_determinant(S_used)
                        logP = -0.5 * logdet - 0.5 * chi_sr * fid_config['moving_window'] + np.log(Hypothesis[m])
                        Hypothesis[m] = logP
                    else:
                        Hypothesis[m] = -np.inf

                maxlogP = np.max(Hypothesis)
                if maxlogP == -np.inf:
                    Hypothesis = np.ones(n_Hypo) * (1.0 / n_Hypo)
                else:
                    indices_not_minusInf = np.where(Hypothesis != -np.inf)[0]
                    logP_not_minusInf = Hypothesis[indices_not_minusInf]
                    P_prime = np.exp(logP_not_minusInf - maxlogP)
                    sum_P_prime = np.sum(P_prime)
                    P_subset = P_prime / sum_P_prime
                    Hypothesis[indices_not_minusInf] = P_subset
                    Hypothesis[Hypothesis == -np.inf] = 0.0

                H_hist.append(Hypothesis.copy())




        N_vals = [sweep_window]
        FID.run_fault_trials(N_vals, sim_config)
        
        return True


    def run_fid(sweep_window, showPlots=False, sim_config=None):
        
        # Setup spacecraft object
        scSim, scObject, simTaskName, \
            simulationTime, simulationTimeStep = FID.setup_basilisk_simulation(simTime=20.0)
        
        # Setup simulation object and configure Mars gravity
        FID.setup_simulation(scSim, simTaskName, scObject)

        # Setup reaction wheels object
        rwFactory, rwStateEffector, numRW = FID.setup_reaction_wheels(scSim, simTaskName, scObject, useJitterSimple=True)

        # Setup guidance and navigation module
        sNavObject, attError = FID.setup_guidance_navigation(scSim, simTaskName)

        # Setup the MRP control module
        sim_config = SimConfig(moving_window=sweep_window)
        mrpControl = FID.setup_mrp_control(scSim, simTaskName, sim_config)

        # Setup motor torque object
        rwMotorTorqueObj = FID.setup_rw_motor_torque(scSim, simTaskName)

        # Setup logging
        numDataPoints = 100
        samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
        logs, fswRwParamMsg = FID.setup_logging_and_rw_config(simTaskName, samplingTime, scSim, rwFactory, rwMotorTorqueObj, attError, sNavObject)

        # Setup the UKF filter (Basilisk inherent)

        ukfSetup = UKFSetup(simulationTimeStep, fswRwParamMsg, rwStateEffector)
        attEstimator, attEstimatorLog = ukfSetup.create_and_setup_filter(scSim, simTaskName)

        result = FID.active_fault_ID(scSim, simTaskName, numRW, sNavObject, attError, sim_config, mrpControl, 
                                     rwMotorTorqueObj,logs, attEstimator, attEstimatorLog, sweep_window=sweep_window
                                )
        return result
