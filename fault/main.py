import argparse
from src.run import run

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--moving', 
        type=int, 
        nargs='*', 
        default=[15]
    )
    args = parser.parse_args()

    for moving_window in args.moving:
        print(f"Running FID with moving window = {moving_window}")
        run(moving_window, terminate=True, show_plots=True)