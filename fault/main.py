import argparse
import random
from src.run import run

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--moving',
        type=int,
        nargs='*',
        default=[1, 5, 10, 15, 20, 25, 30, 35, 40],
        help="List of moving window sizes to sweep over"
    )
    parser.add_argument(
        '--monte_carlo',
        action='store_true',
        help="If set, runs multiple trials per window size with randomly sampled true modes"
    )
    parser.add_argument(
        '--n_trials',
        type=int,
        default=10,
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
        for moving_window in args.moving:
            print(f"\n[Monte Carlo] Running {args.n_trials} FID trials for window = {moving_window}")
            for trial in range(args.n_trials):
                sampled_true_mode = random.choice([0, 1, 2, 3, -1])
                print(f"  Trial {trial+1}/{args.n_trials} | True Mode: {sampled_true_mode}")
                run(moving_window, terminate=False, true_mode=sampled_true_mode, show_plots=True)
    else:
        for moving_window in args.moving:
            print(f"\n[Single Run] Running FID with window = {moving_window} and true mode = {args.true_mode}")
            run(moving_window, terminate=False, true_mode=args.true_mode, show_plots=True)
