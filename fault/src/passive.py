import numpy as np
from scipy.linalg import cholesky, lu

def log_determinant(S):
    """
    Compute the log determinant of matrix S using LU decomposition,
    handling sign of determinant correctly.
    """
    P, L, U = lu(S)
    diagU = np.diag(U)
    signU = np.prod(np.sign(diagU))
    logDet = np.sum(np.log(np.abs(diagU)))
    if signU < 0:
        logDet += np.log(-1)  # complex log if negative determinant
    return logDet


def passive_fault_id(inertialAttFilterLog_dict, moving_window=10, alpha=0.05, crit=0.8):
    """
    Perform passive fault identification offline on a collection of filter logs.

    Args:
        inertialAttFilterLog_dict: dict of filter logs, each having attributes
            'innovation' (Nx x measurement_dim array)
            and 'cov_S' (Nx x measurement_dim x measurement_dim array)
        moving_window: size of sliding window for hypothesis updates
        alpha: significance level for chi-square thresholds
        crit: critical threshold for belief to identify fault

    Returns:
        H_hist: list of hypothesis belief vectors over time
        hypotheses: list of hypothesis keys corresponding to beliefs
    """
    hypotheses = list(inertialAttFilterLog_dict.keys())
    n_hypo = len(hypotheses)

    # Measurement dimension (assumes all same shape)
    ny = inertialAttFilterLog_dict[hypotheses[0]].innovation.shape[1]

    # Initialize equally likely hypothesis vector
    Hypothesis = np.ones(n_hypo) / n_hypo

    # Buffers for innovations and covariances per hypothesis
    inno_hist = {h: [] for h in hypotheses}
    S_hist = {h: [] for h in hypotheses}

    # Store hypothesis history
    H_hist = [Hypothesis.copy()]

    # Degrees of freedom and chi-square thresholds
    df = ny
    # Use percentiles of chi-square distribution as thresholds
    chi_crit_up = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (1 - alpha / 2))) / moving_window
    chi_crit_lo = (np.percentile(np.random.chisquare(df * moving_window, 100000), 100 * (alpha / 2))) / moving_window

    num_steps = inertialAttFilterLog_dict[hypotheses[0]].innovation.shape[0]

    for k in range(num_steps):
        # Append current innovations and covariances to buffers
        for idx, h in enumerate(hypotheses):
            inno_hist[h].append(inertialAttFilterLog_dict[h].innovation[k])
            S_hist[h].append(inertialAttFilterLog_dict[h].cov_S[k])

            # Maintain moving window size
            if len(inno_hist[h]) > moving_window:
                inno_hist[h].pop(0)
                S_hist[h].pop(0)

        # Wait until enough samples collected
        if k < moving_window - 1:
            H_hist.append(Hypothesis.copy())
            continue

        # Update log likelihoods
        logL = np.full(n_hypo, -np.inf)
        for idx, h in enumerate(hypotheses):
            inno_window = np.vstack(inno_hist[h])  # shape (moving_window, ny)
            S_window = np.array(S_hist[h])         # shape (moving_window, ny, ny)

            # Construct big block diagonal covariance matrix S_big
            S_big = np.zeros((ny * moving_window, ny * moving_window))
            for j in range(moving_window):
                S_big[j*ny:(j+1)*ny, j*ny:(j+1)*ny] = S_window[j].reshape(ny, ny)

            # Flatten innovation vector for the window
            inno_flat = inno_window.flatten()

            # Compute whitened chi-square statistic
            try:
                L = cholesky(S_big, lower=True)
                whitened = np.linalg.solve(L, inno_flat)
                chi_sr = np.sum(whitened**2) / moving_window
            except np.linalg.LinAlgError:
                chi_sr = np.inf  # treat as invalid if S_big not positive definite

            # Check chi-square acceptance window and update log likelihood
            if chi_crit_lo <= chi_sr <= chi_crit_up:
                log_det_S = log_determinant(S_big)
                epsilon = 1e-15  # a tiny positive number to avoid log(0)
                logL[idx] = -0.5 * log_det_S - 0.5 * chi_sr * moving_window + np.log(Hypothesis[idx] + epsilon)
            else:
                logL[idx] = -np.inf

        max_logL = np.max(logL)
        if max_logL == -np.inf:
            # All hypotheses rejected, reset to uniform
            Hypothesis = np.ones(n_hypo) / n_hypo
        else:
            # Normalize probabilities to avoid underflow
            logL -= max_logL
            prob = np.exp(logL)
            Hypothesis = prob / np.sum(prob)

        H_hist.append(Hypothesis.copy())

    return H_hist, hypotheses
