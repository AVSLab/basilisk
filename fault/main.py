import argparse
import random
import numpy as np
from src.run import run
from src.utils.plots import plot_failure_rate

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--moving',
        type=lambda s: [int(x) for x in s.strip('[]').split(',')],
        default=[1, 5, 10, 15, 20, 25, 30, 35, 40],
        help="List of moving window sizes to sweep over" # example: "[1,20,40]" make sure to have the " " symbols
    )
    parser.add_argument(
        '--monte_carlo',
        action='store_true',
        help="If set, runs multiple trials per window size with randomly sampled true modes"
    )
    parser.add_argument(
        '--n_trials',
        type=int,
        default=200,
        help="Number of Monte Carlo trials per sweep window (if --monte_carlo is set)"
    )
    parser.add_argument(
        '--true_mode',
        type=int,
        default=1,
        help="True fault mode to use in non-Monte Carlo runs"
    )

    args = parser.parse_args()

    if args.monte_carlo:
        windows = []
        failure_rates = []

        for moving_window in args.moving:
            print(f"\n[Monte Carlo] Running {args.n_trials} FID trials for window = {moving_window}")
            num_failures = 0

            for trial in range(args.n_trials):
                np.random.seed(trial)
                random.seed(trial)
                sampled_true_mode = random.choice([0, 1, 2, 3, -1])
                # print(f"  Trial {trial+1}/{args.n_trials} | True Mode: {sampled_true_mode}")
                result = run(moving_window, true_mode=sampled_true_mode, show_plots=False)

                if result["correct"]:
                    # print(" → Success")
                    num_failures += 0
                else:
                    # print(" → Failure")
                    num_failures += 1

            windows.append(moving_window)
            failure_rate = 100 * num_failures / args.n_trials
            print(f"  -> Failure Rate @ window {moving_window}: {failure_rate:.2f}%")
            failure_rates.append(failure_rate)

        # Plot after all trials
        plot_failure_rate(windows, failure_rates)

    else:
        for moving_window in args.moving:
            print(f"\n[Single Run] Running FID with window = {moving_window} and true mode = {args.true_mode}")
            result = run(moving_window, true_mode=args.true_mode, show_plots=True)
            print(result)
