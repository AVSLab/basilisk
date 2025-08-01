import argparse
from src.run import run

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sweep', 
        type=int, 
        nargs='*', 
        default=[10]
    )
    args = parser.parse_args()

    for sweep_window in args.sweep:
        print(f"Running FID with sweep window = {sweep_window}")
        run(sweep_window, show_plots=True)