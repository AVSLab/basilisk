import numpy as np
from scipy.stats import chi2


class FIDConfig:
    def __init__(self, moving_window=10, alpha=0.05, crit=0.8, ny=3, hypotheses=None):
        self.moving_window = moving_window
        self.alpha = alpha
        self.crit = crit
        self.ny = ny
        self.hypotheses = hypotheses if hypotheses is not None else ["nominal", "fault1", "fault2", "fault3"]
        self.num_hypotheses = len(self.hypotheses)
        self.df = ny
        self.chi_crit = chi2.ppf(1 - alpha, moving_window * ny) / moving_window
        self.chi_crit_up = chi2.ppf(1 - alpha / 2, moving_window * ny) / moving_window
        self.chi_crit_lo = chi2.ppf(alpha / 2, moving_window * ny) / moving_window


class FIDState:
    def __init__(self, config: FIDConfig):
        self.config = config
        self.t_hist = []
        self.H_hist = []
        self.hypothesis = np.ones(config.num_hypotheses) / config.num_hypotheses
        self.fail = 0
        self.id_mode = None
        self.k_end = None
        self.running = True


def compute_chi_whitened(inno_window, S_window, config):
    mw = config.moving_window
    chi_sr = 0.0
    for j in range(mw):
        S_jj = S_window[j]
        inno_jj = inno_window[j]
        try:
            L = np.linalg.cholesky(S_jj)
            whitened = np.linalg.solve(L, inno_jj)
        except np.linalg.LinAlgError:
            return np.inf
        chi_sr += np.sum(whitened ** 2)
    return chi_sr / mw


def update_hypothesis(fid_state: FIDState, inno_buffer: dict, S_buffer: dict):
    config = fid_state.config
    logP = np.full(config.num_hypotheses, -np.inf)

    for idx, key in enumerate(config.hypotheses):
        if key not in inno_buffer or key not in S_buffer:
            continue
        if len(inno_buffer[key]) < config.moving_window:
            continue

        inno_window = inno_buffer[key][-config.moving_window:]
        S_window = S_buffer[key][-config.moving_window:]

        chi_sr = compute_chi_whitened(inno_window, S_window, config)

        if config.chi_crit_lo <= chi_sr <= config.chi_crit_up:
            logP[idx] = -0.5 * config.moving_window * chi_sr + np.log(fid_state.hypothesis[idx])

    maxlogP = np.max(logP)
    if maxlogP == -np.inf:
        fid_state.hypothesis[:] = 1.0 / config.num_hypotheses
    else:
        logP -= maxlogP
        P = np.exp(logP)
        fid_state.hypothesis = P / np.sum(P)

    fid_state.H_hist.append(fid_state.hypothesis.copy())


def check_termination(fid_state: FIDState, k: int, true_mode: int):
    config = fid_state.config
    H = fid_state.hypothesis
    crit = config.crit

    if true_mode == -1:
        if np.max(H) < crit:
            fid_state.fail = 0
            fid_state.id_mode = -1
            fid_state.k_end = k
            return True
    else:
        if np.max(H) < crit:
            fid_state.fail = 1
            fid_state.k_end = k
            return True
        true_H = H[config.hypotheses.index(true_mode) if isinstance(true_mode, str) else true_mode]
        false_H = np.delete(H, config.hypotheses.index(true_mode) if isinstance(true_mode, str) else true_mode)
        if np.max(false_H) > crit:
            fid_state.fail = 1
            fid_state.id_mode = np.argmax(H)
            fid_state.k_end = k
            return True
        if true_H > crit:
            fid_state.fail = 0
            fid_state.id_mode = config.hypotheses.index(true_mode) if isinstance(true_mode, str) else true_mode
            fid_state.k_end = k
            return True

    return False
