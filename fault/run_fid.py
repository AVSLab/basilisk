import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.fswAlgorithms import attTrackingError, mrpFeedback
from Basilisk.simulation import spacecraft, simpleNav

from src.ukf import ukf

from scipy.stats import chi2


def run_fid(sim_config, fid_config, true_mode, noise_parameters):
    """
    Execute a fault identification trial with the given simulation and FID configuration.

    Args:
        sim_config (dict): Configuration parameters for the simulation
        fid_config (dict): Configuration parameters for fault identification
        true_mode (int): Index representing the true fault mode

    Returns:
        tuple: (updated_sim_config, updated_fid_config)
    """

    # Determine stopping point for FID
    k_stop = int(25 + fid_config['moving_window'] * 2.0)

    # Extract timing parameters
    start_time = sim_config['start_time']
    inc_time = sim_config['inc_time']
    end_time = sim_config['end_time']

    # Override end_time for multi-trial setup
    if sim_config['N_trials'] > 1:
        end_time = start_time + k_stop + 1

    # Update the sim_config with modified end_time
    sim_config['end_time'] = end_time

        # Initialization
    nx = noise_parameters['L'].shape[0]
    ny = noise_parameters['H'].shape[0]
    nw = noise_parameters['Q'].shape[0]
    nv = noise_parameters['R'].shape[0]
    t_span = np.arange(start_time, end_time + inc_time, inc_time)
    t_hist = [t_span[0]]

    # Generate nominal initial state
    x_i_nominal = set_x_init(start_time)

    # Add Gaussian noise using initial covariance
    mean = np.zeros(nx)
    cov = noise_parameters['cov_i']
    x_i = x_i_nominal.copy()
    x_i[6:] += np.random.multivariate_normal(mean, cov)

    x_hist = [x_i]

    # Initialize measurement history
    y_hist = [np.zeros(ny)]

    # Extract MRP and omega from current state
    sigma_BR = x_i[6:9]   # body to reference attitude (MRP)
    omega_BR_B = x_i[9:12]  # angular rate in body frame

    # Use Basilisk's control modules
    tracking_error = attTrackingError.attTrackingErrorConfig()
    feedback_ctrl = mrpFeedback.mrpFeedbackConfig()

    # Set gains consistent with sim.constants.controller_K and controller_P
    feedback_ctrl.K = sim_config.get('controller_K', (1/6)**2 / 5)
    feedback_ctrl.P = sim_config.get('controller_P', 1/6)

    # Compute control torque (this would normally be handled through BSK message passing)
    # We'll stub in the torque logic for now as a placeholder
    ctrl_torque = -feedback_ctrl.K * sigma_BR - feedback_ctrl.P * omega_BR_B

    # Control metadata (stub or ideal targets for now)
    ctrl_mode = 0
    r_LMO = np.zeros(3)  # placeholder
    r_GMO = np.zeros(3)
    BN = np.eye(3)
    w_BR_Body = omega_BR_B  # can refine later

    # Save to histories
    rlmo_hist = [r_LMO]
    rgmo_hist = [r_GMO]
    bn_hist = [BN]
    ctrlmode_hist = [ctrl_mode]
    u_hist = [ctrl_torque]
    running_fid_flag = True

    # FID output initialization
    fid_config['outputs'] = {
        'true_mode': true_mode,
        'id_mode': np.nan,
        'fail': 0
    }

    df = ny
    m = fid_config['moving_window']

    chi_crit = chi2.ppf(1 - fid_config['alpha'], m * ny) / m
    chi_crit_up = chi2.ppf(1 - fid_config['alpha'] / 2, df * m) / m
    chi_crit_lo = chi2.ppf(fid_config['alpha'] / 2, df * m) / m

    # Initialize 4 Hypotheses
    n_Hypo = fid_config['N_hypo']
    Hypothesis = np.ones(n_Hypo) * (1.0 / n_Hypo)

    x_i_pos = x_hist[-1]  # equivalent to x_hist(:,end)

    mu_H_pos = np.tile(x_i_pos.reshape(-1, 1), (1, n_Hypo))
    cov_H_pos = np.tile(noise_parameters['cov_i'][:, :, np.newaxis], (1, 1, n_Hypo))
    mu_H_pri = np.copy(mu_H_pos)
    cov_H_pri = np.copy(cov_H_pos)

    H_hist = [Hypothesis]
    mu_H_pos_hist = [mu_H_pos]
    cov_H_pos_hist = [cov_H_pos]
    mu_H_pri_hist = [mu_H_pri]
    cov_H_pri_hist = [cov_H_pri]

    inno_H_pri = np.zeros((ny, n_Hypo))
    inno_H_pri_hist = [inno_H_pri]

    H = noise_parameters['H']
    cov_i = noise_parameters['cov_i']
    S_H_pri = np.tile(H @ cov_i @ H.T[:, :, np.newaxis], (1, 1, n_Hypo))
    S_H_pri_hist = [S_H_pri]


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    # Define the simulation inertia
    I = [10.0, 0., 0.,
         0., 5.0, 0.,
         0., 0., 7.5]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # Create navigation module
    nav = simpleNav.SimpleNav()
    nav.ModelTag = "simpleNav"

    # Add to task
    scSim.AddModelToTask(simTaskName, nav)

    # Connect state output from spacecraft to navigation input
    nav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    # Setup control
    mrp_ctrl = mrpFeedback.mrpFeedbackConfig()
    mrp_ctrl.K = feedback_ctrl.K
    mrp_ctrl.P = feedback_ctrl.P
    mrp_ctrl.Ki = 0.0

    # Main time loop
    for k in range(len(t_span) - 1):
        t1 = t_span[k]
        t2 = t_span[k + 1]
        u_old = u_hist[-1]

        # Set simulation stop time for this interval
        scSim.ConfigureStopTime(macros.sec2nano(t2 - start_time))
        scSim.ExecuteSimulation()

        # Run the Basilisk integration
        scSim.ExecuteSimulation()

        # Read translational state
        trans_msg = nav.transOutMsg.read()
        r_BN_N = trans_msg.r_BN_N
        v_BN_N = trans_msg.v_BN_N

        # Read attitude state
        att_msg = nav.attOutMsg.read()
        sigma_BN = att_msg.sigma_BN
        omega_BN_B = att_msg.omega_BN_B

        x_next = np.concatenate([
            r_BN_N,
            v_BN_N,
            sigma_BN,
            omega_BN_B
        ])
        x_hist.append(x_next)

        # Simulate measurement
        v = np.random.multivariate_normal(np.zeros(nv), noise_parameters['R'])
        y_next = noise_parameters['H'] @ x_next[6:] + v
        y_hist.append(y_next)

        # Fault ID Filtering step
        if running_fid_flag:
            Y = y_hist[-1]

            for m in range(n_Hypo):
                mu_old = mu_H_pos[:, m]
                cov_old = cov_H_pos[:, :, m]

                # Placeholder call to UKF — will define filters.ukf later
                mu_pri, cov_pri, mu_pos, cov_pos, y_pri, cov_S = ukf(
                    sim_config, t1, t2, Y, mu_old, cov_old, u_old, m
                )

                mu_H_pos[:, m] = mu_pos
                cov_H_pos[:, :, m] = cov_pos
                inno_H_pri[:, m] = Y - y_pri
                S_H_pri[:, :, m] = cov_S

        # Store filtering data
        mu_H_pos_hist.append(mu_H_pos.copy())
        cov_H_pos_hist.append(cov_H_pos.copy())
        inno_H_pri_hist.append(inno_H_pri.copy())
        S_H_pri_hist.append(S_H_pri.copy())

        # Hypothesis update
        if k >= fid_config['moving_window']:
            # Log-likelihoods for each hypothesis
            logL = np.zeros(n_Hypo)

            for m in range(n_Hypo):
                # Extract stacked innovation vector over moving window
                inno_used = np.vstack([
                    inno_H_pri_hist[-i - 1][:, m].reshape(-1, 1)
                    for i in reversed(range(fid_config['moving_window']))
                ])

                # Extract stacked block-diagonal innovation covariance
                tmp = [
                    S_H_pri_hist[-i - 1][:, :, m]
                    for i in reversed(range(fid_config['moving_window']))
                ]

                S_used = np.block([
                    [tmp[i] if i == j else np.zeros_like(tmp[0])
                    for j in range(fid_config['moving_window'])]
                    for i in range(fid_config['moving_window'])]
                )

                # Compute Mahalanobis distance
                try:
                    chi = float(inno_used.T @ np.linalg.inv(S_used) @ inno_used) / fid_config['moving_window']
                except np.linalg.LinAlgError:
                    chi = np.inf

                # Reshape innovation vector into ny × moving_window
                inno_whiten = inno_used.reshape(ny, fid_config['moving_window'])

                for jj in range(fid_config['moving_window']):
                    inno_jj = inno_whiten[:, jj]
                    S_jj = S_used[ny * jj : ny * (jj + 1), ny * jj : ny * (jj + 1)]

                    try:
                        # Cholesky decomposition (lower triangular)
                        L = np.linalg.cholesky(S_jj)
                        inno_whiten_jj = np.linalg.solve(L, inno_jj)  # L \ inno_jj
                        inno_whiten[:, jj] = inno_whiten_jj
                    except np.linalg.LinAlgError:
                        inno_whiten[:, jj] = np.full_like(inno_jj, np.inf)
        
                # Compute whitened chi-square statistic
                chi_sr = np.sum(inno_whiten**2) / fid_config['moving_window']
                # print(f"Chi diff (unwhitened - whitened): {chi - chi_sr}")

                # Flatten whitened innovations
                inno_used_whiten = inno_whiten.reshape(ny * fid_config['moving_window'], 1)

                # Whitened covariance is identity
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

        # Extract current state at t2
        x_current = x_hist[-1]
        sigma_BR = x_current[6:9]
        omega_BR_B = x_current[9:12]
        r_BN_N = x_current[0:3]  # Position in inertial frame

        # Control gains
        K = sim_config.get('controller_K', (1/6)**2 / 5)
        P = sim_config.get('controller_P', 1/6)

        # Define orbital positions for virtual reference targets
        r_LMO_inertial = np.array([4000.0, 0.0, 0.0])  # [km]
        r_GMO_inertial = np.array([17000.0, 0.0, 0.0])  # [km]

        # Relative vectors to reference targets
        r_LMO = r_LMO_inertial - r_BN_N
        r_GMO = r_GMO_inertial - r_BN_N

        # Control mode based on proximity
        d_LMO = np.linalg.norm(r_LMO)
        d_GMO = np.linalg.norm(r_GMO)
        ctrl_mode = int(d_LMO > d_GMO)

        # Placeholder for body-to-nav rotation matrix (could later be from attitude estimation)
        BN = np.eye(3)

        # Nominal control
        u_apply = -K * sigma_BR - P * omega_BR_B

        # Active fault-ID weighted control
        if running_fid_flag and fid_config['action_flag']:
            u_weighted = np.zeros(3)
            for m in range(n_Hypo):
                mu_m = mu_H_pos[:, m]
                sigma_m = mu_m[6:9]
                omega_m = mu_m[9:12]
                u_m = -K * sigma_m - P * omega_m
                u_weighted += Hypothesis[m] * u_m
            u_apply = u_weighted

        # Log control and metadata
        t_hist.append(t2)
        u_hist.append(u_apply)
        rlmo_hist.append(r_LMO)
        rgmo_hist.append(r_GMO)
        ctrlmode_hist.append(ctrl_mode)
        bn_hist.append(BN)

        # === Termination condition ===
        if running_fid_flag:
            terminate_flag, k_end, fail, id_mode = event_termination(true_mode, k, k_stop, fid_config, Hypothesis)
            if terminate_flag:
                running_fid_flag = False
                fid_config['outputs']['k_end'] = k_end + 1
                fid_config['outputs']['fail'] = fail
                fid_config['outputs']['id_mode'] = id_mode


        fid_config['outputs'].update({
            't_hist': t_hist,
            'x_hist': x_hist,
            'y_hist': y_hist,
            'u_hist': u_hist,
            'H_hist': H_hist,
            'rlmo_hist': rlmo_hist,
            'rgmo_hist': rgmo_hist,
            'bn_hist': bn_hist,
            'ctrlmode_hist': ctrlmode_hist,
            'mu_H_pos_hist': mu_H_pos_hist,
            'cov_H_pos_hist': cov_H_pos_hist,
            'inno_H_pri_hist': inno_H_pri_hist,
            'S_H_pri_hist': S_H_pri_hist
        })

    return sim_config, fid_config


def set_x_init(start_time):
    """
    Define the nominal initial state vector based on start_time.
    This mimics satsym.set_x_init from MATLAB.

    Returns:
        np.ndarray: 12-element state vector
    """
    r = np.array([7000.0, 0.0, 0.0])  # km
    v = np.array([0.0, 7.5, 0.0])     # km/s
    sigma = np.array([0.0, 0.0, 0.0]) # MRP
    omega = np.array([0.0, 0.0, 0.0]) # rad/s
    return np.concatenate((r, v, sigma, omega))


def ukf(sim_config, t1, t2, Y, mu_old, cov_old, u_old, fault_mode):
    """
    Placeholder UKF function for hypothesis update.

    Returns:
        mu_pri: predicted mean
        cov_pri: predicted covariance
        mu_pos: posterior mean
        cov_pos: posterior covariance
        y_pri: predicted measurement
        cov_S: innovation covariance
    """
    nx = len(mu_old)
    ny = len(Y)

    # Dummy values until UKF is implemented
    mu_pri = mu_old.copy()
    cov_pri = cov_old.copy()
    mu_pos = mu_old.copy()
    cov_pos = cov_old.copy()
    y_pri = Y.copy()
    cov_S = np.eye(ny) * 1e-3

    return mu_pri, cov_pri, mu_pos, cov_pos, y_pri, cov_S


def log_determinant(S):
    """
    Compute the log-determinant of a matrix using LU decomposition.
    """
    try:
        P, L, U = np.linalg.lu(S)  # Note: numpy doesn't have lu directly
    except AttributeError:
        # Use scipy if available
        from scipy.linalg import lu
        P, L, U = lu(S)

    diagU = np.diag(U)
    signU = np.prod(np.sign(diagU))
    logDet = np.sum(np.log(np.abs(diagU)))

    if signU < 0:
        logDet += np.log(-1)  # add imaginary part

    return logDet


def event_termination(true_mode, k, k_stop, fid_config, Hypothesis):
    terminate_flag = False
    k_end = None
    fail = None
    id_mode = None
    crit = fid_config['crit']

    # 1. Timeout condition
    if k == k_stop:
        if true_mode == -1:
            if np.max(Hypothesis) < crit:
                k_end = k
                fail = 0
                id_mode = -1
                terminate_flag = True
                return terminate_flag, k_end, fail, id_mode
        else:
            if np.max(Hypothesis) < crit:
                k_end = k
                fail = 1
                terminate_flag = True
                return terminate_flag, k_end, fail, id_mode

    # 2. False hypothesis identified
    if true_mode >= 0:
        false_indices = list(range(len(Hypothesis)))
        if true_mode < len(Hypothesis):
            false_indices.pop(true_mode)  # exclude true hypothesis index

        false_Hypothesis = Hypothesis[false_indices]
        if np.max(false_Hypothesis) > crit:
            detected = np.argmax(Hypothesis)
            id_mode = detected
            k_end = k
            fail = 1
            terminate_flag = True
            return terminate_flag, k_end, fail, id_mode

    # 3. True hypothesis identified
    if true_mode >= 0 and Hypothesis[true_mode] > crit:
        id_mode = np.argmax(Hypothesis)
        k_end = k
        fail = 0
        terminate_flag = True
        return terminate_flag, k_end, fail, id_mode

    return terminate_flag, k_end, fail, id_mode

