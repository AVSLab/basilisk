from src.core import run
from src.sweeping import sweep_moving_window

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--sweep', action='store_true')
    args = parser.parse_args()

    if args.sweep:
        N_vals, fail_rate, avg_delay = sweep_moving_window()
    else:
        run(True, 5)
