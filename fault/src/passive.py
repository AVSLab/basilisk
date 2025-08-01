import numpy as np
from scipy.linalg import cholesky, lu

def passive_fault_id(inertialAttFilterLog_dict, moving_window, alpha=0.05, crit=0.95, terminate=True, true_mode = 0):
    """
    Perform passive fault identification offline on a collection of filter logs.

    Args:
        inertialAttFilterLog_dict: dict of filter logs, each having attributes
            'innovation' (Nx x measurement_dim array)
            and 'cov_S' (Nx x measurement_dim x measurement_dim array)
        moving_window: size of sliding window for hypothesis updates
        alpha: significance level for chi-square thresholds
        crit: critical threshold for belief to identify fault
        terminate: boolean indicating whether to terminate early on identification

    Returns:
        H_hist: list of hypothesis belief vectors over time
        hypotheses: list of hypothesis keys corresponding to beliefs
        k_end: timestep where identification terminated (None if not terminated)
        fail: 1 if identification failed, 0 if succeeded
        id_mode: identified mode
    """
    hypotheses = list(inertialAttFilterLog_dict.keys())
    n_hypo = len(hypotheses)
    ny = inertialAttFilterLog_dict[hypotheses[0]].innovation.shape[1]

    Hypothesis = np.ones(n_hypo) / n_hypo
    inno_hist = {h: [] for h in hypotheses}
    S_hist = {h: [] for h in hypotheses}
    H_hist = [Hypothesis.copy()]

    df = ny
    chi_crit_up = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (1 - alpha / 2))) / moving_window
    chi_crit_lo = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (alpha / 2))) / moving_window

    num_steps = inertialAttFilterLog_dict[hypotheses[0]].innovation.shape[0]
    k_stop = int(25 + moving_window * 2.0)
    k_end = None
    fail = None
    id_mode = None

    for k in range(num_steps):
        for h in hypotheses:
            inno_hist[h].append(inertialAttFilterLog_dict[h].innovation[k])
            S_hist[h].append(inertialAttFilterLog_dict[h].cov_S[k])
            if len(inno_hist[h]) > moving_window:
                inno_hist[h].pop(0)
                S_hist[h].pop(0)

        if k < moving_window - 1:
            H_hist.append(Hypothesis.copy())
            continue

        logL = np.full(n_hypo, -np.inf)
        for idx, h in enumerate(hypotheses):
            inno_window = np.vstack(inno_hist[h])
            S_window = np.array(S_hist[h])

            S_big = np.zeros((ny * moving_window, ny * moving_window))
            for j in range(moving_window):
                S_big[j*ny:(j+1)*ny, j*ny:(j+1)*ny] = S_window[j].reshape(ny, ny)

            inno_flat = inno_window.flatten()

            try:
                L = cholesky(S_big, lower=True)
                whitened = np.linalg.solve(L, inno_flat)
                chi_sr = np.sum(whitened**2) / moving_window
            except np.linalg.LinAlgError:
                chi_sr = np.inf

            if chi_crit_lo <= chi_sr <= chi_crit_up:
                log_det_S = log_determinant(S_big)
                epsilon = 1e-15
                logL[idx] = -0.5 * log_det_S - 0.5 * chi_sr * moving_window + np.log(Hypothesis[idx] + epsilon)
            else:
                logL[idx] = -np.inf

        max_logL = np.max(logL)
        if max_logL == -np.inf:
            Hypothesis = np.ones(n_hypo) / n_hypo
        else:
            logL -= max_logL
            prob = np.exp(logL)
            Hypothesis = prob / np.sum(prob)

        H_hist.append(Hypothesis.copy())

        if terminate:
            terminate_flag, k_end, fail, id_mode = check_termination(k, Hypothesis, crit)
            
            # Incorporate true_mode check
            if terminate_flag:
                # Check if the identified mode matches the true mode
                if id_mode == true_mode:
                    fail = 0  # success
                else:
                    fail = 1  # failure (wrong mode identified)
                break

        if not terminate and k == k_stop:
            k_end = k
            fail = 1
            id_mode = None
            break

    return H_hist, hypotheses, k_end, fail, id_mode

def log_determinant(S):
    P, L, U = lu(S)
    diagU = np.diag(U)
    signU = np.prod(np.sign(diagU))
    logDet = np.sum(np.log(np.abs(diagU)))
    if signU < 0:
        logDet += np.log(-1)
    return logDet

def check_termination(k, Hypothesis, crit):
    terminate_flag = False
    k_end = None
    fail = None
    id_mode = None

    for i, val in enumerate(Hypothesis):
        if val > crit:
            terminate_flag = True
            k_end = k
            fail = 0
            id_mode = i
            return terminate_flag, k_end, fail, id_mode

    return terminate_flag, k_end, fail, id_mode
