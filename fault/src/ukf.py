import numpy as np

def unscented_transform(sigmas, Wm, Wc, noise_cov):
    mean = np.dot(Wm, sigmas)
    kmax, n = sigmas.shape
    cov = np.zeros((n, n))
    for k in range(kmax):
        y = sigmas[k] - mean
        cov += Wc[k] * np.outer(y, y)
    cov += noise_cov
    return mean, cov

def generate_sigma_points(x, P, alpha=1e-3, beta=2, kappa=0):
    n = x.shape[0]
    lambda_ = alpha**2 * (n + kappa) - n
    sigma_points = np.zeros((2 * n + 1, n))
    Wm = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wc = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = lambda_ / (n + lambda_) + (1 - alpha**2 + beta)

    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    sigma_points[0] = x
    for i in range(n):
        sigma_points[i + 1]     = x + sqrt_P[:, i]
        sigma_points[i + 1 + n] = x - sqrt_P[:, i]

    return sigma_points, Wm, Wc

def ukf_predict(f, x, P, Q):
    sigma_points, Wm, Wc = generate_sigma_points(x, P)
    sigmas_f = np.array([f(sp) for sp in sigma_points])
    x_pred, P_pred = unscented_transform(sigmas_f, Wm, Wc, Q)
    return x_pred, P_pred, sigmas_f, sigma_points, Wm, Wc

def ukf_update(h, x_pred, P_pred, sigmas_f, sigma_points, Wm, Wc, R, y):
    sigmas_h = np.array([h(sp) for sp in sigma_points])
    y_pred, P_yy = unscented_transform(sigmas_h, Wm, Wc, R)

    P_xy = np.zeros((x_pred.size, y_pred.size))
    for i in range(sigma_points.shape[0]):
        dx = sigmas_f[i] - x_pred
        dy = sigmas_h[i] - y_pred
        P_xy += Wc[i] * np.outer(dx, dy)

    K = np.dot(P_xy, np.linalg.inv(P_yy))
    x_post = x_pred + np.dot(K, (y - y_pred))
    P_post = P_pred - K @ P_yy @ K.T
    return x_post, P_post, y_pred, P_yy

def ukf(sim_config, t1, t2, y, x, P, u, fault_mode):
    inc_time = sim_config['inc_time']
    Q = sim_config['Q']
    R = sim_config['R']

    def f(x):
        # simple identity dynamics with additive noise
        return x

    def h(x):
        H = sim_config['H']
        return H @ x[6:]

    x_pred, P_pred, sigmas_f, sigma_points, Wm, Wc = ukf_predict(f, x, P, Q)
    x_post, P_post, y_pred, S = ukf_update(h, x_pred, P_pred, sigmas_f, sigma_points, Wm, Wc, R, y)
    return x_pred, P_pred, x_post, P_post, y_pred, S
