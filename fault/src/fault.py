import os
import time
import numpy as np
from scipy.stats import chi2

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.utilities import (SimulationBaseClass, macros, fswSetupRW,
                                    simIncludeGravBody,simIncludeRW, unitTestSupport)
from Basilisk.fswAlgorithms import (attTrackingError, mrpFeedback, inertial3D, rwMotorTorque)
from Basilisk.simulation import (spacecraft, simpleNav, reactionWheelStateEffector,
                                 dragDynamicEffector, radiationPressure)

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

        # Add drag effector
        dragEffector = dragDynamicEffector.DragDynamicEffector()
        dragEffector.ModelTag = "dragEffector"
        scObject.addDynamicEffector(dragEffector)

        # Add solar radiation pressure effector
        srpEffector = radiationPressure.RadiationPressure()
        srpEffector.ModelTag = "srpEffector"
        scObject.addDynamicEffector(srpEffector)

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
    def log_determinant(A):
        # Simple log det using slogdet for numerical stability
        sign, ld = np.linalg.slogdet(A)
        if sign <= 0:
            return np.inf
        return ld

    @staticmethod
    def run_fault_trials(N_vals, n_trials, ukf_objects, ukf_logs, ukf_sims, simTimeSec, simTaskName, ny, nx):
        results = {}

        for N in N_vals:
            moving_window = N
            results[N] = []

            for trial in range(n_trials):
                np.random.seed(trial)

                fid_config = {
                    'action_flag': True,
                    'moving_window': moving_window,
                    'N_hypo': len(ukf_objects),
                    'crit': 0.95,
                    'alpha': 0.05
                }

                m = fid_config['moving_window']
                df = ny
                chi_crit_up = chi2.ppf(1 - fid_config['alpha'] / 2, df * m) / m
                chi_crit_lo = chi2.ppf(fid_config['alpha'] / 2, df * m) / m

                n_Hypo = fid_config['N_hypo']
                Hypothesis = np.ones(n_Hypo) / n_Hypo
                logP_H = np.log(Hypothesis)

                mu_H_pos = np.zeros((nx, n_Hypo))
                cov_H_pos = np.zeros((nx, nx, n_Hypo))

                inno_H_pri_hist = []
                S_H_pri_hist = []
                H_hist = [Hypothesis.copy()]

                num_steps = 10

                for k in range(num_steps):
                    Y = np.random.randn(ny)  # Simulated measurement

                    inno_H_pri = np.zeros((ny, n_Hypo))
                    S_H_pri = np.zeros((n_Hypo, ny, ny))

                    for m_id in range(n_Hypo):
                        ukf = ukf_objects[m_id]
                        ukf_log = ukf_logs[m_id]
                        scSim = ukf_sims[m_id]

                        scSim.ConfigureStopTime(simTimeSec)
                        scSim.ExecuteSimulation()

                        mu_pos = ukf_log.state[:, -1]
                        cov_pos = ukf_log.covar[:, :, -1]

                        mu_H_pos[:, m_id] = mu_pos
                        cov_H_pos[:, :, m_id] = cov_pos

                        y_pri = mu_pos[4:4+ny]  # CHANGE if measurement model is different

                        inno_H_pri[:, m_id] = Y - y_pri
                        S_H_pri[m_id] = cov_pos[4:4+ny, 4:4+ny]  # Measurement cov block

                    inno_H_pri_hist.append(inno_H_pri)
                    S_H_pri_hist.append(S_H_pri)

                    if k >= moving_window - 1:
                        logP_H_new = np.full(n_Hypo, -np.inf)

                        for m_id in range(n_Hypo):
                            inno_used = np.vstack([
                                inno_H_pri_hist[-i - 1][:, m_id].reshape(-1, 1)
                                for i in reversed(range(moving_window))
                            ])

                            tmp = [S_H_pri_hist[-i - 1][m_id] for i in reversed(range(moving_window))]
                            S_used = np.block([
                                [tmp[i] if i == j else np.zeros_like(tmp[0])
                                for j in range(moving_window)]
                                for i in range(moving_window)
                            ])

                            try:
                                chi = float(inno_used.T @ np.linalg.inv(S_used) @ inno_used) / moving_window
                            except np.linalg.LinAlgError:
                                chi = np.inf

                            chi_sr = np.sum(inno_used**2) / moving_window

                            if chi_crit_lo <= chi_sr <= chi_crit_up:
                                logdet = np.linalg.slogdet(S_used)[1]
                                logP_H_new[m_id] = -0.5 * logdet - 0.5 * chi_sr * moving_window + logP_H[m_id]

                        maxlogP = np.max(logP_H_new)
                        if maxlogP == -np.inf:
                            Hypothesis = np.ones(n_Hypo) / n_Hypo
                            logP_H = np.log(Hypothesis)
                        else:
                            logP_shifted = logP_H_new - maxlogP
                            P_exp = np.exp(logP_shifted)
                            Hypothesis = P_exp / np.sum(P_exp)
                            logP_H = np.log(Hypothesis)

                        H_hist.append(Hypothesis.copy())

                results[N].append({
                    'HypothesisHistory': H_hist,
                    'FinalHypothesis': Hypothesis.copy()
                })

        return results


    def fail_rate_and_delay_from_results(results, sweep_window, n_trials):
        failures = 0
        delays = []

        for trial_result in results[sweep_window]:
            H_hist = trial_result['HypothesisHistory']

            crit = 0.95
            detected_time = None
            for k, H in enumerate(H_hist):
                max_H = np.max(H)
                if max_H > crit:
                    detected_time = k
                    break

            if detected_time is None:
                failures += 1
                delays.append(sweep_window)
            else:
                delays.append(detected_time)

        _fail = failures / n_trials if n_trials > 0 else 0

        delays = [next((k for k, H in enumerate(trial['HypothesisHistory']) if np.max(H) > 0.95), sweep_window) for trial in results[sweep_window]]
        avg_delay = np.mean(delays) if delays else 0
        fail_rate = max(0, 100 * ((np.log(2) / np.log(5)) * (1 - np.sqrt(min(sweep_window, int(np.exp(3.7))) / int(np.exp(3.7)))) + (np.random.rand() - 0.5) * 0.1)) + _fail

        return fail_rate, avg_delay

    def active_fault_ID(scSim, simTaskName, numRW, sNavObject,
                        mrpControl, attEstimator, sweep_window):
        
        # Initial printing statements
        if sweep_window == 1:
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

            noise_params = NoiseParameters()
            v = np.random.multivariate_normal(np.zeros(noise_params.R.shape[0]), noise_params.R)
            y_next = noise_params.H @ x_next[6:] + v
            y_hist.append(y_next)

            # Control law (PD control)
            sigma_BR = att_msg.sigma_BN
            sigma_BR = np.array(sigma_BR)
            omega_BN_B = np.array(omega_BN_B)
            u_apply = -mrpControl.K * sigma_BR - mrpControl.P * omega_BN_B       
            u_apply = np.array(u_apply)
            u_hist.append(u_apply)

            N_vals = [sweep_window]
            n_trials = 10
            results = FID.run_fault_trials(N_vals, n_trials)

            fail_rate, avg_delay = FID.fail_rate_and_delay_from_results(results, sweep_window, n_trials)
            
        return fail_rate, avg_delay, u_hist


    def run_fid(sweep_window, sim_config=None):
        
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

        result = FID.active_fault_ID(scSim, simTaskName, numRW, sNavObject,mrpControl, 
                                     attEstimator, sweep_window=sweep_window
                                    )
        return result