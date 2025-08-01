# plots.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros


def plot_attitude_error(timeData, dataSigmaBR):
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')


def plot_filter_result_sigma(filter_key, timeData, state, state_est, cov_est):
    timeData = timeData[1:]
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    for idx, ax in enumerate(axs):
        color = unitTestSupport.getLineColor(idx, 3)
        ax.plot(timeData, state[1:, idx], color=color, label=rf'$x_{{{idx+1}}}$')
        ax.plot(timeData, state_est[1:, idx], color=color, linestyle='--', label=rf'$\hat{{x}}_{{{idx+1}}}$')
        std = 6 * np.sqrt(cov_est[1:, idx])
        ax.fill_between(timeData, state_est[1:, idx] - std, state_est[1:, idx] + std, color=color, alpha=0.3, label=r'$\pm6\sigma$')
        ax.set_ylabel(rf'$x_{{{idx+1}}}$')
        ax.legend(loc='upper right', fontsize='small')
    axs[-1].set_xlabel('Time [min]')
    fig.suptitle(f"[{filter_key} filter] result", fontsize=14)


def plot_filter_result_omega(filter_key, timeData, state, state_est, cov_est):
    timeData = timeData[1:]
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    for idx, ax in enumerate(axs):
        color = unitTestSupport.getLineColor(idx, 3)
        ax.plot(timeData, state[1:, idx], color=color, label=rf'$x_{{{idx+3}}}$')
        ax.plot(timeData, state_est[1:, idx], color=color, linestyle='--', label=rf'$\hat{{x}}_{{{idx+3}}}$')
        std = 6 * np.sqrt(cov_est[1:, idx])
        ax.fill_between(timeData, state_est[1:, idx] - std, state_est[1:, idx] + std, color=color, alpha=0.3, label=r'$\pm6\sigma$')
        ax.set_ylabel(rf'$x_{{{idx+3}}}$')
        ax.legend(loc='upper right', fontsize='small')
    axs[-1].set_xlabel('Time [min]')
    fig.suptitle(f"[{filter_key} filter] result", fontsize=14)


def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    plt.figure(2)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx], '--', color=unitTestSupport.getLineColor(idx, numRW),
                 label=rf'$\hat u_{{s,{idx}}}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    plt.figure(2)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx], '--', color=unitTestSupport.getLineColor(idx, numRW),
                 label=rf'$\hat u_{{s,{idx}}}$')
        plt.plot(timeData, dataRW[idx], color=unitTestSupport.getLineColor(idx, numRW),
                 label=rf'$u_{{s,{idx}}}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')


def plot_rate_error(timeData, dataOmegaBR):
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx], color=unitTestSupport.getLineColor(idx, 3),
                 label=rf'$\omega_{{BR,{idx}}}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s)')


def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=rf'$\Omega_{{{idx}}}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM)')


def plot_filter_chisquare(dataChiSquare):
    p = 0.05
    dof = 3
    chi_ub = chi2.ppf(1 - 0.5 * p, dof)
    chi_lb = chi2.ppf(0.5 * p, dof)
    plt.figure()
    plt.scatter(np.arange(len(dataChiSquare)), dataChiSquare, color="black", s=10, label=r"$\chi^2$")
    plt.axhline(y=chi_ub, color='r', linestyle='--', label=r"$\chi^2$ upper threshold")
    plt.axhline(y=chi_lb, color='b', linestyle='--', label=r"$\chi^2$ lower threshold")
    plt.legend(loc='upper right')
