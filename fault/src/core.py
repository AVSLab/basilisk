import os
import numpy as np
from run_fid import run_fid

def run(show_plots, moving_window):
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

    sim_config = {
        'N_trials': 2,
        'save_data': True,
        'rlevel': 'hig',
        'targetPath': os.path.join(os.getcwd(), 'data', 'dataset1c', 'rlevel_hig'),
        'inc_time': 1
    }

    fid_config = {
        'action_flag': True,
        'moving_window': moving_window,
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

    np.random.seed(0)
    true_mode = np.random.randint(-1, 4)
    if true_mode < -1:
        true_mode = -1
    sim_config['start_time'] = np.random.randint(0, 7102)
    sim_config['end_time'] = sim_config['start_time'] + 1000

    sim_config, fid_config = run_fid(sim_config, fid_config, true_mode, noise_parameters)

    return sim_config, fid_config
