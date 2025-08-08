import numpy as np
# from scipy.linalg import cholesky, lu

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
        terminate (obsolete): boolean indicating whether to terminate early on identification

    Returns:
        timeData: time points corresponding to hypothesis belief
        H_hist: list of hypothesis belief vectors over time
        hypotheses: list of hypothesis keys corresponding to beliefs
        k_end: timestep where identification terminated (None if not terminated)
        id_mode: identified mode
    """
    hypotheses = list(inertialAttFilterLog_dict.keys())
    n_hypo = len(hypotheses)
    ny = inertialAttFilterLog_dict[hypotheses[0]].innovation.shape[1]
    timeData = inertialAttFilterLog_dict[hypotheses[0]].times()/(1E+9)

    Hypothesis = np.ones(n_hypo) / n_hypo
    H_hist = []

    df = ny
    chi_crit_up = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (1 - alpha / 2))) / moving_window
    chi_crit_lo = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (alpha / 2))) / moving_window
    k_stop = int(25 + moving_window * 2)

    # pre-compute chi_square and det(cov_S) for all UKFs
    chi_square_hist = np.zeros((k_stop, n_hypo))
    det_covS_hist = np.zeros((k_stop, n_hypo))
    for idx, h in enumerate(hypotheses):
        inno_hist = inertialAttFilterLog_dict[h].innovation[:k_stop, :]
        S_hist = inertialAttFilterLog_dict[h].cov_S[:k_stop, :]
        for i in range(inno_hist.shape[0]):
            inno_i = inno_hist[i, :].reshape(ny, 1)
            cov_S_i = S_hist[i, :].reshape(ny, ny)
            try:
                invS = np.linalg.inv(cov_S_i)
                chi_square_i = (inno_i.T @ invS @ inno_i).item()
                chi_square_hist[i, idx] = chi_square_i
                det_covS_hist[i, idx] = np.linalg.det(cov_S_i)
            except np.linalg.LinAlgError:
                chi_square_hist[i, idx] = np.nan
                det_covS_hist[i, idx] = np.nan

    # time loop for hypothesis belief update
    for k in range(k_stop):
        begin_idx = k-moving_window+1
        if(begin_idx >= 0):
            # get the data in moving window
            data_chi_square = chi_square_hist[begin_idx:k+1, :]
            data_det_covS = det_covS_hist[begin_idx:k+1, :]
    
            # compute logits for each hypothesis
            logits = np.zeros(n_hypo)
            for idx in range(n_hypo):
                chi_square = data_chi_square[:, idx]
                chi_square_avg = np.mean(chi_square)
                if(chi_square_avg > chi_crit_up or chi_square_avg < chi_crit_lo or np.isnan(chi_square_avg)):
                    logits[idx] = -np.inf # rejection
                else:
                    det_covS = data_det_covS[:, idx]
                    sum_log_det_covS = np.sum(np.log(det_covS))
                    sum_chi_square = np.sum(chi_square)
                    if(Hypothesis[idx] > 0): 
                        logL = -0.5*sum_log_det_covS -0.5*sum_chi_square + np.log(Hypothesis[idx])
                    else:
                        logL = -np.inf
                    logits[idx] = logL

            # use masked softmax to convert logits into probability
            Hypothesis = softmax_with_inf_mask(logits)
            H_hist.append(Hypothesis.copy())
        else:
            H_hist.append(Hypothesis.copy())

        # identify fault hypo if a belief is greater than crit
        for idx, val in enumerate(Hypothesis):
            if val > crit:
                id_mode = idx
                k_end = k
                timeData = timeData[:len(H_hist)]
                return timeData, H_hist, hypotheses, k_end, id_mode
            
    # return unknown fault        
    id_mode = -1
    k_end = k_stop
    timeData = timeData[:len(H_hist)]
    return timeData, H_hist, hypotheses, k_end, id_mode


def softmax_with_inf_mask(logits: np.ndarray) -> np.ndarray:
    """
    Compute a softmax over `logits`, but:
      - any ±inf entries get probability 0
      - if all logits are non-finite (e.g. all -inf), return uniform probabilities
    """
    logits = np.asarray(logits, dtype=float)
    n = logits.size
    # Mask finite entries
    valid = np.isfinite(logits)
    probs = np.zeros_like(logits, dtype=float)

    # If no finite logits (e.g. all -inf), return uniform
    if not np.any(valid):
        return np.full(n, 1.0/n)

    # Only exponentiate the finite ones
    finite = logits[valid]
    shifted = finite - np.max(finite)
    exps = np.exp(shifted)
    denom = exps.sum()

    if denom == 0:
        # extremely small values → fallback to uniform over valids
        probs[valid] = 1.0 / valid.sum()
    else:
        probs[valid] = exps / denom

    return probs
