import os
import numpy as np

class SimConfig:
    def __init__(self, moving_window):
        self.N_trials = 2
        self.save_data = True
        self.rlevel = 'hig'
        self.targetPath = os.path.join(os.getcwd(), 'data', 'dataset1c', 'rlevel_hig')

        # Controller gains - can be overridden or passed in
        self.controller_K = (1/6)**2 / 5  # default proportional gain K
        self.controller_P = 1/6            # default derivative gain P

        # Store moving window
        self.moving_window = moving_window

class NoiseParameters:
    def __init__(self):
        self.cov_i = np.block([
            [1e-4 * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), 1e-6 * np.eye(3)]
        ])
        self.L = np.vstack((np.zeros((3, 3)), np.eye(3)))
        self.Q = 1e-9 * np.eye(3)
        self.H = np.eye(6)
        self.R = 0.05 * np.block([
            [1e-4 * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), 1e-6 * np.eye(3)]
        ])

class FilterData:
    def __init__(self):
        # UKF tuning parameters
        self.alpha = 0.02
        self.beta = 2.0
        self.kappa = 0.0
        self.switchMag = 1.2

        # Initial state vector: attitude MRP (3) + angular velocity (3)
        self.stateInit = [0.1, 0.2, -0.3, 0.001, -0.01, 0.03]

        # Initial covariance matrix (6x6) flattened in row-major order
        self.covarInit = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        ]

        # Process noise covariance matrix Q (6x6), flattened
        qNoise = np.identity(6)
        qNoise[0:3, 0:3] *= 0.0017 ** 2    # attitude noise variance
        qNoise[3:6, 3:6] *= 0.00017 ** 2   # gyro bias noise variance
        self.qNoise = qNoise.flatten().tolist()
