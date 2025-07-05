import numpy as np
from run_fid import run_fid

def sweep_moving_window():
    N_vals = [1] + list(range(5, 45, 5))
    fail_rate = []
    avg_delay = []

    # Fixed noise parameters
    noise_parameters = {
        'cov_i': np.block([
            [1e-4 * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), 1e-6 * np.eye(3)]
        ]),
        'L': np.vstack((np.zeros((3, 3)), np.eye(3))),
        'Q': 1e-9 * np.eye(3),
        'H': np.eye(6),
        'R': 0.05 * np.block([
            [1e-4 * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), 1e-6 * np.eye(3)]
        ]),
    }

    for N in N_vals:
        n_trials = 10
        failures = 0
        delays = []

        for trial in range(n_trials):
            np.random.seed(trial)

            sim_config = {
                'N_trials': 1,
                'save_data': False,
                'rlevel': 'hig',
                'inc_time': 1,
                'start_time': np.random.randint(0, 7102),
            }
            sim_config['end_time'] = sim_config['start_time'] + 1000

            fid_config = {
                'action_flag': True,
                'moving_window': N,
                'N_hypo': 4,
                'crit': 0.95,
                'alpha': 0.05
            }

            if sim_config['rlevel'] == "hig":
                noise_parameters['R'] *= 4.0
            elif sim_config['rlevel'] == "mid":
                noise_parameters['R'] *= 1.0
            else:
                noise_parameters['R'] *= 0.25

            true_mode = np.random.randint(-1, 4)
            sim_config, fid_config = run_fid(sim_config, fid_config, true_mode, noise_parameters)
            outputs = fid_config['outputs']

            if outputs['fail']:
                failures += 1
            else:
                delays.append(outputs['k_end'])

        fail_percent = 100 * failures / n_trials
        mean_delay = np.mean(delays) if delays else 0

        fail_rate.append(fail_percent)
        avg_delay.append(mean_delay)

    return N_vals, fail_rate, avg_delay
