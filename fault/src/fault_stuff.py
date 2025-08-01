from .fid import FIDConfig, FIDState, update_hypothesis, check_termination

# FID initialization
config = FIDConfig(moving_window=sweep_window)
fid_state = FIDState(config)

# Set up FID buffers with integer keys
inno_buffer = {m: [] for m in range(4)}
S_buffer = {m: [] for m in range(4)}
true_mode = 0  # nominal mode index

mode_names = {
    0: "nominal",
    1: "fault1",
    2: "fault2",
    3: "fault3",
}

# Log innovations and covariance for each hypothesis using integer keys
for m, name in mode_names.items():
    log = inertialAttFilterLog_dict[name]
    if log.innovation.shape[0] > i:
        inno_buffer[m].append(log.innovation[i])
        S_buffer[m].append(log.cov_S[i])
        # Maintain moving window size
        if len(inno_buffer[m]) > sweep_window:
            inno_buffer[m] = inno_buffer[m][-sweep_window:]
            S_buffer[m] = S_buffer[m][-sweep_window:]

# Only update FID after enough samples
if i >= sweep_window:
    update_hypothesis(fid_state, inno_buffer, S_buffer)
    terminated = check_termination(fid_state, i, true_mode)
    if terminated:
        print(f"FID terminated at step {i}:")
        print(f"  Identified mode: {fid_state.id_mode}")
        print(f"  Failed: {fid_state.fail}")
        print(f"  Hypothesis: {fid_state.hypothesis}")