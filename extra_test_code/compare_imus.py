#!/usr/bin/env python3
"""Compare two IMU CSV recordings by timestamp and plot angles + difference.

Usage:
  python compare_imus.py --csv1 path/to/first.csv --csv2 path/to/second.csv [--col COL] [--sr 222.22] [--out out.png] [--show]

The script aligns both files to a common uniform timebase over their overlapping interval
by linear interpolation, plots both angle traces and their difference, and prints summary
statistics (mean abs, RMS, max abs).
"""
from pathlib import Path
import argparse
import sys
import csv
from datetime import datetime


def normalize_quaternion(quaternion):
    values = np.asarray(quaternion, dtype=float)
    magnitude = np.linalg.norm(values)
    if magnitude == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return values / magnitude


def quaternion_conjugate(quaternion):
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([w, -x, -y, -z], dtype=float)


def quaternion_multiply(left, right):
    w1, x1, y1, z1 = normalize_quaternion(left)
    w2, x2, y2, z2 = normalize_quaternion(right)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def relative_quaternion(reference, target):
    return normalize_quaternion(quaternion_multiply(quaternion_conjugate(reference), target))


def quaternion_angle_degrees(quaternion):
    w, x, y, z = normalize_quaternion(quaternion)
    angle_radians = 2.0 * np.arctan2(np.linalg.norm([x, y, z]), abs(w))
    return float(np.degrees(np.clip(angle_radians, 0.0, np.pi)))
import numpy as np
import matplotlib.pyplot as plt


def read_time_and_col(path: Path, time_cols=('rel_time_s', 'epoch_time', 'system_time_iso'), col_name='flexion_deg'):
    # Read CSV into dict of columns (similar helper as other scripts)
    with open(path, newline='', encoding='utf-8') as fh:
        reader = list(csv.reader(fh))
    if not reader:
        raise ValueError(f"Empty CSV: {path}")

    # detect which row is the header: look for known tokens
    header_row_idx = None
    for i, row in enumerate(reader[:5]):
        joined = ','.join(row).upper()
        if 'ORIENTATION' in joined or 'SYSTEM TIME' in joined or 'REL_TIME' in joined or 'EPOCH_TIME' in joined:
            header_row_idx = i
            break
    if header_row_idx is None:
        # default to second row if first looks like metadata
        header_row_idx = 1 if len(reader) > 1 else 0

    header = reader[header_row_idx]
    data_rows = reader[header_row_idx + 1:]
    cols = {h: [] for h in header}
    for row in data_rows:
        # pad or trim rows to header length
        if len(row) < len(header):
            row = row + [''] * (len(header) - len(row))
        for h, v in zip(header, row[:len(header)]):
            cols[h].append(v)

    # find time column
    time_col = None
    for tname in time_cols:
        if tname in cols:
            time_col = tname
            break

    def try_parse_iso(s):
        if s is None:
            return None
        s = s.strip()
        if not s:
            return None
        # normalize trailing Z
        if s.endswith('Z'):
            s = s[:-1] + '+00:00'
        try:
            return datetime.fromisoformat(s)
        except Exception:
            # try common alternatives
            for fmt in ('%Y-%m-%d %H:%M:%S.%f', '%Y/%m/%d %H:%M:%S.%f', '%d-%m-%YT%H:%M:%S.%f'):
                try:
                    return datetime.strptime(s, fmt)
                except Exception:
                    continue
        return None

    # If no known time column, fall back to first column numeric values
    if time_col is None:
        first_col = list(cols.keys())[0]
        # try numeric
        try:
            times = np.array(cols[first_col], dtype=float)
            # assume these are seconds (relative or epoch); keep as-is
        except Exception:
            # try parsing as ISO timestamps
            parsed = [try_parse_iso(x) for x in cols[first_col]]
            if all(p is not None for p in parsed):
                times = np.array([p.timestamp() for p in parsed], dtype=float)
            else:
                raise ValueError(f"No time column found in {path}; expected one of {time_cols} and first column not numeric or timestamp")
    else:
        # convert to numpy floats seconds (absolute epoch seconds when possible)
        if time_col == 'system_time_iso':
            parsed = [try_parse_iso(x) for x in cols[time_col]]
            if not any(parsed):
                raise ValueError(f"Could not parse ISO times in column {time_col} for {path}")
            times = np.array([p.timestamp() for p in parsed], dtype=float)
        else:
            # numeric epoch or relative times
            times = np.array(cols[time_col], dtype=float)
            # detect if these look like milliseconds (very large numbers)
            if np.median(times) > 1e11:
                times = times / 1000.0

    # If flexion column exists, return it
    if col_name in cols:
        vals = np.array(cols[col_name], dtype=float)
        return times, vals

    # Attempt to detect quaternion groups: look for headers that include 'ORIENTATION W/X/Y/Z'
    headers = list(cols.keys())
    hdr_up = [h.upper() for h in headers]
    # gather candidate indices for each component, prefer numeric columns that include '('
    w_candidates = [i for i, h in enumerate(hdr_up) if 'ORIENTATION W' in h]
    x_candidates = [i for i, h in enumerate(hdr_up) if 'ORIENTATION X' in h]
    y_candidates = [i for i, h in enumerate(hdr_up) if 'ORIENTATION Y' in h]
    z_candidates = [i for i, h in enumerate(hdr_up) if 'ORIENTATION Z' in h]

    def pick_preferred(cands):
        if not cands:
            return None
        for i in cands:
            if '(' in headers[i]:
                return i
        return cands[0]

    sensors = []
    # pair candidates by occurrence order — assume groupings appear repeatedly
    for wi in w_candidates:
        xi = pick_preferred([c for c in x_candidates if c > wi - 5])
        yi = pick_preferred([c for c in y_candidates if c > wi - 5])
        zi = pick_preferred([c for c in z_candidates if c > wi - 5])
        # if not found nearby, pick any available
        if xi is None:
            xi = pick_preferred(x_candidates)
        if yi is None:
            yi = pick_preferred(y_candidates)
        if zi is None:
            zi = pick_preferred(z_candidates)
        if xi is not None and yi is not None and zi is not None:
            sensors.append((wi, xi, yi, zi))

    if len(sensors) < 2:
        raise ValueError(f"Column '{col_name}' not found and could not detect two quaternion sensors in {path}")

    # read quaternion values row-by-row and skip rows with missing values
    q_arrays = []
    for (wi, xi, yi, zi) in sensors[:2]:
        w_list = []
        x_list = []
        y_list = []
        z_list = []
        time_list = []
        n_rows = len(next(iter(cols.values())))
        for i in range(n_rows):
            try:
                wv = cols[headers[wi]][i]
                xv = cols[headers[xi]][i]
                yv = cols[headers[yi]][i]
                zv = cols[headers[zi]][i]
            except Exception:
                # missing index
                continue
            try:
                wfv = float(wv)
                xfv = float(xv)
                yfv = float(yv)
                zfv = float(zv)
            except Exception:
                # skip rows where conversion fails or empty strings
                continue
            # take corresponding time if available and numeric
            time_val = None
            if isinstance(times, np.ndarray) and i < len(times):
                time_val = times[i]
            else:
                try:
                    time_val = float(list(cols.values())[0][i])
                except Exception:
                    time_val = None
            if time_val is None:
                continue
            w_list.append(wfv)
            x_list.append(xfv)
            y_list.append(yfv)
            z_list.append(zfv)
            time_list.append(time_val)

        if len(w_list) == 0:
            raise ValueError(f"No valid quaternion rows found for sensor in {path}")

        q = np.vstack([np.array(w_list), np.array(x_list), np.array(y_list), np.array(z_list)]).T
        q_arrays.append((np.array(time_list), q))

    if len(q_arrays) < 2:
        raise ValueError(f"Could not extract two valid quaternion sensors from {path}")

    # align by common times: take intersection of timestamps
    t0, q0 = q_arrays[0]
    t1b, q1b = q_arrays[1]
    # pick times present in first sensor and within range of second
    mask = (t0 >= t1b[0]) & (t0 <= t1b[-1])
    t_common = t0[mask]
    if len(t_common) == 0:
        raise ValueError(f"No overlapping quaternion timestamps in {path}")

    # interpolate q1b to t_common by linear interp per component
    from numpy import interp
    q1_interp = np.column_stack([
        np.interp(t_common, t1b, q1b[:, c]) for c in range(4)
    ])
    q0_sel = q0[mask]

    n = len(t_common)
    angles = np.empty(n, dtype=float)
    for i in range(n):
        rel = relative_quaternion(q1_interp[i], q0_sel[i])
        angles[i] = quaternion_angle_degrees(rel)

    return t_common, angles


def infer_sample_rate(t: np.ndarray):
    if len(t) < 2:
        return None
    dt = np.diff(t)
    # avoid zeros
    dt = dt[dt > 1e-9]
    if len(dt) == 0:
        return None
    return 1.0 / np.median(dt)


def compare_and_plot(csv1: Path, csv2: Path, col='flexion_deg', out: Path = None, show: bool = False, dump_matches: Path = None, swap: bool = False):
    t1, a1 = read_time_and_col(csv1, col_name=col)
    t2, a2 = read_time_and_col(csv2, col_name=col)

    # Match timestamps by hour:minute:second and choose nearest millisecond in the other file.
    # Convert to integer milliseconds
    t1 = np.asarray(t1, dtype=float)
    t2 = np.asarray(t2, dtype=float)
    if len(t1) == 0 or len(t2) == 0:
        raise ValueError('Empty time series')

    t1_ms = np.round(t1 * 1000).astype(np.int64)
    t2_ms = np.round(t2 * 1000).astype(np.int64)

    # helper to convert ms -> datetime for h:m:s test
    from datetime import datetime as _dt
    def ms_to_hms(ms):
        return _dt.fromtimestamp(ms / 1000.0).hour, _dt.fromtimestamp(ms / 1000.0).minute, _dt.fromtimestamp(ms / 1000.0).second

    # ensure t2_ms sorted
    order2 = np.argsort(t2_ms)
    t2_ms_sorted = t2_ms[order2]
    a2_sorted = a2[order2]

    t_common_list = []
    a1_list = []
    a2_list = []
    import bisect
    for i, ms in enumerate(t1_ms):
        # find nearest index in t2_ms_sorted
        j = bisect.bisect_left(t2_ms_sorted, ms)
        candidates = []
        if j < len(t2_ms_sorted):
            candidates.append(j)
        if j - 1 >= 0:
            candidates.append(j - 1)
        if not candidates:
            continue
        # pick best candidate by abs diff
        best = min(candidates, key=lambda idx: abs(int(t2_ms_sorted[idx]) - int(ms)))
        # require hour/minute/second to match
        if ms_to_hms(ms) != ms_to_hms(int(t2_ms_sorted[best])):
            continue
        # accept pairing
        t_common_list.append((ms + int(t2_ms_sorted[best])) / 2000.0)
        a1_list.append(a1[i])
        a2_list.append(a2_sorted[best])

    if len(t_common_list) == 0:
        raise ValueError('No timestamp matches by hour:minute:second between recordings')

    t_common = np.array(t_common_list, dtype=float)
    a1_interp = np.array(a1_list, dtype=float)
    a2_interp = np.array(a2_list, dtype=float)

    # optional: swap which is considered reference/target (useful if mapping inverted)
    if swap:
        a1_interp, a2_interp = a2_interp, a1_interp

    # optional: dump matched pairs for inspection
    if dump_matches is not None:
        import csv as _csv
        with open(dump_matches, 'w', newline='', encoding='utf-8') as _fh:
            w = _csv.writer(_fh)
            w.writerow(['time_s', f'{csv1.name}_angle', f'{csv2.name}_angle'])
            for ti, va, vb in zip(t_common, a1_interp, a2_interp):
                w.writerow([f'{ti:.6f}', f'{va:.6f}', f'{vb:.6f}'])

    diff = a1_interp - a2_interp

    mean_abs = np.mean(np.abs(diff))
    rms = np.sqrt(np.mean(diff**2))
    max_abs = np.max(np.abs(diff))

    # plot
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(14, 8), gridspec_kw={'height_ratios': [3, 1]})
    ax1.plot(t_common, a1_interp, label=f'{csv1.name}', lw=1.5)
    ax1.plot(t_common, a2_interp, label=f'{csv2.name}', lw=1.5)
    ax1.set_ylabel('Angle (deg)')
    ax1.legend(loc='upper right')
    ax1.grid(linestyle=':', linewidth=0.7)

    ax2.plot(t_common, diff, color='tab:red', lw=1.0)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Diff (deg)')
    ax2.grid(linestyle=':', linewidth=0.7)

    # annotate metrics on the figure
    stats_text = f'Mean abs: {mean_abs:.3f}°\nRMS: {rms:.3f}°\nMax abs: {max_abs:.3f}°'
    fig.suptitle('IMU Angle Comparison', fontsize=16)
    fig.text(0.02, 0.02, stats_text, fontsize=10, bbox=dict(facecolor='white', alpha=0.8))

    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    if out is None:
        out = Path('compare_imus.png')
    fig.savefig(out, dpi=150)
    if show:
        plt.show()
    plt.close(fig)

    print(f'Comparison saved to: {out}')
    print(f'Mean abs diff: {mean_abs:.4f}°, RMS: {rms:.4f}°, Max abs: {max_abs:.4f}°')


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--csv1', type=Path, required=True)
    p.add_argument('--csv2', type=Path, required=True)
    p.add_argument('--col', default='flexion_deg', help='Column name for angle (default: flexion_deg)')
    p.add_argument('--out', type=Path, help='Output PNG file')
    p.add_argument('--show', action='store_true')
    p.add_argument('--dump-matches', type=Path, help='Write matched timestamp pairs to CSV')
    p.add_argument('--swap', action='store_true', help='Swap the two matched angle series (useful when sensor mapping is reversed)')
    args = p.parse_args(argv)

    compare_and_plot(args.csv1, args.csv2, col=args.col, out=args.out, show=args.show, dump_matches=args.dump_matches, swap=args.swap)


if __name__ == '__main__':
    main()
