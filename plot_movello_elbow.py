#!/usr/bin/env python3
"""Plot elbow joint angle from a Delsys/Movella live CSV like the example image.

Usage:
  python plot_delsys_elbow.py [--csv PATH] [--out PATH] [--show]

If --csv is omitted the most recent `live_stream_*.csv` in the cwd is used.
"""
from pathlib import Path
import argparse
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt


def find_latest_live_csv(folder: Path) -> Path:
    files = list(folder.glob("Xsens_Reading\\Live_stream\\live_stream_*.csv"))
    if not files:
        raise FileNotFoundError("no live_stream_*.csv files found in current folder")
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0]


def _read_csv_columns(csv_path: Path):
    # returns a dict of column -> numpy array (floats where possible)
    with open(csv_path, newline='', encoding='utf-8') as fh:
        reader = csv.reader(fh)
        header = next(reader)
        cols = {h: [] for h in header}
        for row in reader:
            if len(row) != len(header):
                # skip malformed rows
                continue
            for h, v in zip(header, row):
                cols[h].append(v)
    # convert numeric columns to float arrays when possible
    for h, vals in list(cols.items()):
        try:
            arr = np.array(vals, dtype=float)
            cols[h] = arr
        except Exception:
            cols[h] = np.array(vals)
    return cols


def plot_elbow(csv_path: Path, out_path: Path = None, show: bool = False, sr: float = None, smooth_ms: float = 0.0):
    cols = _read_csv_columns(csv_path)

    # prefer relative time column; fall back to epoch_time
    if 'rel_time_s' in cols:
        t = cols['rel_time_s'].astype(float)
    elif 'epoch_time' in cols:
        epoch = cols['epoch_time'].astype(float)
        t = (epoch - epoch[0]).astype(float)
    else:
        raise ValueError("CSV missing time columns ('rel_time_s' or 'epoch_time')")

    if 'flexion_deg' in cols:
        angle = cols['flexion_deg'].astype(float)
    else:
        raise ValueError("CSV does not contain 'flexion_deg' column")

    # No baseline shading by default — plot data directly from CSV

    # Optional resample to uniform timebase to match device sample rate and
    # apply a short moving-average smoothing if requested to reduce jitter.
    t_plot = t
    angle_plot = angle
    if sr is not None and sr > 0 and len(t) > 1:
        t0, t1 = float(t[0]), float(t[-1])
        dt = 1.0 / float(sr)
        t_uniform = np.arange(t0, t1 + dt / 2.0, dt)
        angle_uniform = np.interp(t_uniform, t, angle)
        # apply moving-average smoothing window
        if smooth_ms and smooth_ms > 0:
            win = int(round((smooth_ms / 1000.0) * sr))
            if win < 1:
                win = 1
            # make odd window
            if win % 2 == 0:
                win += 1
            # simple convolution smoothing
            kernel = np.ones(win) / win
            angle_uniform = np.convolve(angle_uniform, kernel, mode='same')
        t_plot = t_uniform
        angle_plot = angle_uniform

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.plot(t_plot, angle_plot, lw=2.6, color='#1f77b4', label='Elbow Joint Angle (Forearm to Triceps)')

    # plotting the angle trace directly (no baseline shading)

    # styling
    ax.set_title('IMU Elbow Joint Angle (Sensor 0: Forearm vs Sensor 1: Triceps)', fontsize=18)
    ax.set_xlabel('Timestamp (s)', fontsize=14)
    ax.set_ylabel('Joint Angle (deg)', fontsize=14)
    ax.grid(which='major', linestyle=':', linewidth=0.8)
    ax.legend(loc='upper right', fontsize=12)
    ax.tick_params(labelsize=12)

    # no baseline annotation requested by user

    # tighten and save
    fig.tight_layout()
    if out_path is None:
        out_path = csv_path.with_suffix('.png')
    fig.savefig(out_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)
    print(f"Saved plot to: {out_path}")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=Path, help='Path to live CSV file')
    parser.add_argument('--out', type=Path, help='Output PNG path')
    parser.add_argument('--show', action='store_true', help='Show plot interactively')
    args = parser.parse_args(argv)

    try:
        csv_path = args.csv if args.csv is not None else find_latest_live_csv(Path.cwd())
    except Exception as e:
        print(f"Error locating CSV: {e}")
        sys.exit(1)

    try:
        plot_elbow(csv_path, args.out, args.show)
    except Exception as e:
        print(f"Error while plotting: {e}")
        raise


if __name__ == '__main__':
    main()
