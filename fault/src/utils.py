# src/utils/filter_setup.py

import numpy as np

def setup_filter_data(filterObject):
    """
    Configure initial settings for the inertialUKF filter object.
    This includes UKF tuning parameters, initial state/covariance,
    and process noise covariance matrix.
    """
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2

    # Initial state: [sigma_BN (3), omega_BN_B (3)]
    filterObject.stateInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Initial covariance matrix (6x6, flattened)
    filterObject.covarInit = [
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    ]

    # Process noise matrix (Q) - 6x6, flattened
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] *= 0.0017 ** 2   # attitude noise
    qNoise[3:6, 3:6] *= 0.00017 ** 2  # gyro bias noise
    filterObject.qNoise = qNoise.flatten().tolist()
