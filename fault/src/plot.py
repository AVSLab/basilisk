import matplotlib.pyplot as plt
import numpy as np


def plot_fid_metrics(sweep_values, fail_rates, avg_delays, save_path=None, show=True):
    """
    Plots failure rate and average delay vs sweep window size.

    Parameters:
    - sweep_values: List of sweep window sizes
    - fail_rates: List of failure rates corresponding to sweep_values
    - avg_delays: List of average delays corresponding to sweep_values
    - save_path: Optional file path to save the plot
    - show: Whether to display the plot
    """
    plt.figure(figsize=(10, 4))

    # Failure rate plot
    plt.subplot(1, 2, 1)
    plt.plot(sweep_values, fail_rates, marker='o', color='red')
    plt.xlabel("Sweep Window Size")
    plt.ylabel("Failure Rate (%)")
    plt.grid(True)

    # Average delay plot
    plt.subplot(1, 2, 2)
    plt.plot(sweep_values, avg_delays, marker='s', color='blue')
    plt.xlabel("Sweep Window Size")
    plt.ylabel("Average Delay")
    plt.grid(True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path)

    if show:
        plt.show()
    else:
        plt.close()


def plot_control_history(u_hist, save_path=None, show=True):
    """
    Plots control torque history over time for each control axis, using sample indices on x-axis.

    Parameters:
    - u_hist: list or array of control vectors (shape [n_steps, n_axes])
    - save_path: optional file path to save the figure
    - show: whether to display the plot
    """

    U = np.array(u_hist)
    t = np.arange(U.shape[0])  # sample indices

    plt.figure(figsize=(8, 4))
    for i in range(U.shape[1]):
        plt.plot(t, U[:, i], label=f'Control Axis {i+1}')

    plt.title('Control Torque History')
    plt.xlabel('Time Step Index')
    plt.ylabel('Control Torque')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path)
    if show:
        plt.show()
    else:
        plt.close()
