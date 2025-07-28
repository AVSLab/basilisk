# FID Evaluation Script

This script evaluates a **Fault Identification (FID)** system over a range of sweep window sizes, recording and plotting key performance metrics.

## What It Does

- Iterates over a list of `sweep_window` values.
- For each sweep size, it runs `FID.run_fid(sweep_window=...)` and collects:
  - Failure rate
  - Average fault detection delay
  - Control input history
- Plots:
  - Failure rate vs. sweep window size
  - Average detection delay vs. sweep window size

## How to Run

### Run with default sweep values

This runs the FID evaluation with the default sweep window sizes:

```bash
python main.py
```

Specify your own list of sweep window sizes:

```bash
python main.py --sweep 2 4 6 8 10
```