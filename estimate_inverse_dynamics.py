import csv
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parent
INPUT_DIR = PROJECT_ROOT / "Xsens_Reading"
OUTPUT_FILE = INPUT_DIR / "inverse_dynamics_results.csv"
CURVES_PLOT_FILE = INPUT_DIR / "inverse_dynamics_moment_curves.png"
SUMMARY_PLOT_FILE = INPUT_DIR / "inverse_dynamics_peak_summary.png"


# Arm26 forearm-hand body properties from arm26.osim.
FOREARM_HAND_MASS_KG = 1.534315
FOREARM_HAND_COM_M = 0.181479
FOREARM_HAND_INERTIA_Z_KG_M2 = 0.020062

# Approximate elbow-to-styloid distance from the model marker set.
LOAD_MOMENT_ARM_M = 0.23559

GRAVITY_M_S2 = 9.80665


@dataclass
class TrialResult:
    source_file: str
    load_kg: float
    peak_moment_nm: float
    mean_moment_nm: float
    peak_flexion_deg: float


def write_motion_mot_file(output_path: Path, time_s: np.ndarray, flexion_deg: np.ndarray):
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("arm26_elbow_motion\n")
        handle.write("version=1\n")
        handle.write(f"nRows={len(time_s)}\n")
        handle.write("nColumns=2\n")
        handle.write("inDegrees=no\n")
        handle.write("endheader\n")
        handle.write("time\tr_elbow_flex\n")

        for current_time, flexion_value in zip(time_s, flexion_deg):
            flexion_radians = np.radians(np.clip(flexion_value, 0.0, 140.0))
            handle.write(f"{current_time:.6f}\t{flexion_radians:.8f}\n")


def parse_load_from_filename(path: Path) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)kg", path.name)
    if not match:
        raise ValueError(f"Could not infer load from filename: {path.name}")
    return float(match.group(1))


def load_trial_csv(path: Path):
    times = []
    flexion_deg = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                times.append(float(row["rel_time_s"]))
                flexion_deg.append(float(row["flexion_deg"]))
            except (KeyError, TypeError, ValueError):
                continue

    if len(times) < 3:
        raise ValueError(f"Not enough samples in {path.name}")

    time_array = np.asarray(times, dtype=float)
    flexion_array = np.asarray(flexion_deg, dtype=float)

    order = np.argsort(time_array)
    time_array = time_array[order]
    flexion_array = flexion_array[order]

    unique_time, unique_index = np.unique(time_array, return_index=True)
    return unique_time, flexion_array[unique_index]


def smooth_signal(values: np.ndarray, window: int = 7) -> np.ndarray:
    if window <= 1 or len(values) < window:
        return values.astype(float, copy=True)

    kernel = np.ones(window, dtype=float) / window
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def estimate_elbow_moment(time_s: np.ndarray, flexion_deg: np.ndarray, load_kg: float):
    flexion_rad = np.radians(flexion_deg)
    flexion_rad = smooth_signal(flexion_rad, window=7)

    if len(time_s) < 3:
        raise ValueError("Need at least 3 samples to estimate dynamics")

    flexion_vel = np.gradient(flexion_rad, time_s)
    flexion_acc = np.gradient(flexion_vel, time_s)

    # Use a scalar approximation for the forearm-hand inertia about the elbow axis.
    forearm_hand_inertia_about_elbow = (
        FOREARM_HAND_INERTIA_Z_KG_M2
        + FOREARM_HAND_MASS_KG * FOREARM_HAND_COM_M**2
    )
    load_inertia_about_elbow = load_kg * LOAD_MOMENT_ARM_M**2
    effective_inertia = forearm_hand_inertia_about_elbow + load_inertia_about_elbow

    # Gravity term for a sagittal-plane elbow flexion model.
    gravity_moment_segment = (
        FOREARM_HAND_MASS_KG
        * GRAVITY_M_S2
        * FOREARM_HAND_COM_M
        * np.sin(flexion_rad)
    )
    gravity_moment_load = load_kg * GRAVITY_M_S2 * LOAD_MOMENT_ARM_M * np.sin(flexion_rad)

    inertial_moment = effective_inertia * flexion_acc
    net_moment = inertial_moment + gravity_moment_segment + gravity_moment_load

    return {
        "time_s": time_s,
        "flexion_deg": np.degrees(flexion_rad),
        "flexion_vel_deg_s": np.degrees(flexion_vel),
        "flexion_acc_deg_s2": np.degrees(flexion_acc),
        "segment_gravity_nm": gravity_moment_segment,
        "load_gravity_nm": gravity_moment_load,
        "inertial_nm": inertial_moment,
        "net_elbow_moment_nm": net_moment,
    }


def process_trial(path: Path) -> TrialResult:
    load_kg = parse_load_from_filename(path)
    time_s, flexion_deg = load_trial_csv(path)
    result = estimate_elbow_moment(time_s, flexion_deg, load_kg)

    peak_index = int(np.argmax(np.abs(result["net_elbow_moment_nm"])))
    return TrialResult(
        source_file=path.name,
        load_kg=load_kg,
        peak_moment_nm=float(result["net_elbow_moment_nm"][peak_index]),
        mean_moment_nm=float(np.mean(result["net_elbow_moment_nm"])),
        peak_flexion_deg=float(np.max(result["flexion_deg"])),
    )


def main():
    csv_files = sorted(INPUT_DIR.glob("*kg_movella_test1.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No '*kg_movella_test1.csv' files found in {INPUT_DIR}")

    all_rows = []
    summaries = []

    for csv_path in csv_files:
        load_kg = parse_load_from_filename(csv_path)
        time_s, flexion_deg = load_trial_csv(csv_path)
        result = estimate_elbow_moment(time_s, flexion_deg, load_kg)
        peak_index = int(np.argmax(np.abs(result["net_elbow_moment_nm"])))

        mot_file = INPUT_DIR / f"{int(load_kg)}kg_arm26_elbow_motion.mot"
        write_motion_mot_file(mot_file, result["time_s"], result["flexion_deg"])

        per_trial_plot_file = INPUT_DIR / f"inverse_dynamics_moment_{int(load_kg)}kg.png"
        fig_trial, ax_trial = plt.subplots(figsize=(11, 6.5))
        fig_trial.suptitle(f"Estimated Elbow Net Moment Over Time - {load_kg:.0f} kg")
        ax_trial.plot(result["time_s"], result["net_elbow_moment_nm"], linewidth=1.9, color="#1f77b4")
        ax_trial.set_xlabel("Time (s)")
        ax_trial.set_ylabel("Net elbow moment (N·m)")
        ax_trial.grid(True, linestyle="--", alpha=0.35)
        ax_trial.text(
            0.02,
            0.98,
            f"Peak: {result['net_elbow_moment_nm'][peak_index]:.2f} N·m\nMean: {np.mean(result['net_elbow_moment_nm']):.2f} N·m",
            transform=ax_trial.transAxes,
            va="top",
            ha="left",
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.85},
        )
        fig_trial.tight_layout()
        fig_trial.savefig(per_trial_plot_file, dpi=180)
        plt.close(fig_trial)

        summaries.append(
            TrialResult(
                source_file=csv_path.name,
                load_kg=load_kg,
                peak_moment_nm=float(result["net_elbow_moment_nm"][peak_index]),
                mean_moment_nm=float(np.mean(result["net_elbow_moment_nm"])),
                peak_flexion_deg=float(np.max(result["flexion_deg"])),
            )
        )

        for i in range(len(time_s)):
            all_rows.append(
                {
                    "source_file": csv_path.name,
                    "load_kg": f"{load_kg:.3f}",
                    "time_s": f"{result['time_s'][i]:.6f}",
                    "flexion_deg": f"{result['flexion_deg'][i]:.6f}",
                    "flexion_vel_deg_s": f"{result['flexion_vel_deg_s'][i]:.6f}",
                    "flexion_acc_deg_s2": f"{result['flexion_acc_deg_s2'][i]:.6f}",
                    "segment_gravity_nm": f"{result['segment_gravity_nm'][i]:.6f}",
                    "load_gravity_nm": f"{result['load_gravity_nm'][i]:.6f}",
                    "inertial_nm": f"{result['inertial_nm'][i]:.6f}",
                    "net_elbow_moment_nm": f"{result['net_elbow_moment_nm'][i]:.6f}",
                }
            )

    with OUTPUT_FILE.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "source_file",
            "load_kg",
            "time_s",
            "flexion_deg",
            "flexion_vel_deg_s",
            "flexion_acc_deg_s2",
            "segment_gravity_nm",
            "load_gravity_nm",
            "inertial_nm",
            "net_elbow_moment_nm",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    fig1, ax1 = plt.subplots(figsize=(11, 6.5))
    fig1.suptitle("Estimated Elbow Net Moment Over Time")
    for csv_path in csv_files:
        load_kg = parse_load_from_filename(csv_path)
        time_s, flexion_deg = load_trial_csv(csv_path)
        result = estimate_elbow_moment(time_s, flexion_deg, load_kg)
        ax1.plot(result["time_s"], result["net_elbow_moment_nm"], linewidth=1.8, label=f"{load_kg:.0f} kg")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Net elbow moment (N·m)")
    ax1.grid(True, linestyle="--", alpha=0.35)
    ax1.legend(title="Dumbbell load")
    fig1.tight_layout()
    fig1.savefig(CURVES_PLOT_FILE, dpi=180)
    plt.close(fig1)

    fig2, ax2 = plt.subplots(figsize=(8.5, 5.5))
    fig2.suptitle("Peak Net Elbow Moment by Load")
    loads = [summary.load_kg for summary in summaries]
    peaks = [summary.peak_moment_nm for summary in summaries]
    ax2.plot(loads, peaks, marker="o", linewidth=2.0)
    for load, peak in zip(loads, peaks):
        ax2.annotate(f"{peak:.1f}", (load, peak), textcoords="offset points", xytext=(0, 8), ha="center")
    ax2.set_xlabel("Load (kg)")
    ax2.set_ylabel("Peak net elbow moment (N·m)")
    ax2.grid(True, linestyle="--", alpha=0.35)
    fig2.tight_layout()
    fig2.savefig(SUMMARY_PLOT_FILE, dpi=180)
    plt.close(fig2)

    print(f"Wrote detailed results to {OUTPUT_FILE}")
    print(f"Saved moment curve plot to {CURVES_PLOT_FILE}")
    print(f"Saved peak summary plot to {SUMMARY_PLOT_FILE}")
    for csv_path in csv_files:
        load_kg = parse_load_from_filename(csv_path)
        print(f"Saved motion file to {INPUT_DIR / f'{int(load_kg)}kg_arm26_elbow_motion.mot'}")
        print(f"Saved per-trial plot to {INPUT_DIR / f'inverse_dynamics_moment_{int(load_kg)}kg.png'}")
    print("\nSummary by trial:")
    for summary in summaries:
        print(
            f"{summary.source_file}: load={summary.load_kg:.1f} kg, "
            f"peak moment={summary.peak_moment_nm:.3f} N·m, "
            f"mean moment={summary.mean_moment_nm:.3f} N·m, "
            f"peak flexion={summary.peak_flexion_deg:.2f} deg"
        )


if __name__ == "__main__":
    main()