import argparse
from src.fault import FID
from src.plot import plot_fid_metrics

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sweep', 
        type=int, 
        nargs='*', 
        default=[1, 5, 10, 15, 20, 25, 30, 35, 40]
    )
    parser.add_argument('--showPlots', action='store_true', help="Display plots")
    args = parser.parse_args()

    sweep_values = args.sweep
    all_fail_rates = []
    all_avg_delays = []
    u_hist = []
    for sweep in args.sweep:
        fail_rate, avg_delay, u = FID.run_fid(sweep_window=sweep)
        all_fail_rates.append(fail_rate)
        all_avg_delays.append(avg_delay)
        u_hist = u

    # Plotting
    plot_fid_metrics(sweep_values=sweep_values,
                    fail_rates=all_fail_rates,
                    avg_delays=all_avg_delays)
