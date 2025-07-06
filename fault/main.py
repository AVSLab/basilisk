import argparse
from src.fault import FID

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sweep', type=int, default=0, help="Set sweep window size (0 for none)")
    parser.add_argument('--showPlots', action='store_true', help="Display plots")
    args = parser.parse_args()

    FID.run_fid(sweep_window=args.sweep, showPlots=args.showPlots)
