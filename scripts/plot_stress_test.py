"""
Generate analysis plots from stress test data.

Usage:
    python scripts/plot_stress_test.py stress_test_results/20260327_140000/
"""

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.patches import Patch
from datetime import datetime, timedelta

# Finger colour palette (consistent across all plots)
FINGER_COLORS = {
    "thumb": "#e74c3c",
    "index": "#3498db",
    "middle": "#2ecc71",
    "ring": "#f39c12",
    "pinky": "#9b59b6",
    "wrist": "#7f8c8d",
}


def joint_color(joint_name):
    finger = joint_name.split("_")[0]
    return FINGER_COLORS.get(finger, "#333333")


def load_data(output_dir):
    readings_path = os.path.join(output_dir, "readings.csv")
    events_path = os.path.join(output_dir, "events.csv")
    meta_path = os.path.join(output_dir, "metadata.json")

    df = pd.read_csv(readings_path)
    df["timestamp"] = pd.to_datetime(df["timestamp"], unit="s")

    events = pd.read_csv(events_path)
    if not events.empty:
        events["timestamp"] = pd.to_datetime(events["timestamp"], unit="s")

    with open(meta_path) as f:
        meta = json.load(f)

    return df, events, meta


# -----------------------------------------------------------------------
# Plot 1: Temperature over time
# -----------------------------------------------------------------------
def plot_temperature(df, meta, output_dir):
    temp_df = df.dropna(subset=["temperature"])
    if temp_df.empty:
        print("  No temperature data to plot.")
        return

    fig, ax = plt.subplots(figsize=(14, 6))

    for joint in temp_df["joint_name"].unique():
        jd = temp_df[temp_df["joint_name"] == joint].sort_values("elapsed_s")
        ax.plot(jd["elapsed_s"] / 3600, jd["temperature"],
                label=joint, color=joint_color(joint), alpha=0.8, linewidth=0.8)

    ax.axhline(meta["max_temp"], color="red", linestyle="--",
               linewidth=1, label=f"Max ({meta['max_temp']}°C)")
    ax.set_xlabel("Time (hours)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("Motor Temperature Over Time")
    ax.legend(fontsize=7, ncol=4, loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "temperature.png"), dpi=150)
    plt.close(fig)
    print("  Saved temperature.png")


# -----------------------------------------------------------------------
# Plot 2: Thermal shutdown analysis
# -----------------------------------------------------------------------
def plot_shutdown_analysis(events, meta, output_dir):
    if events.empty or "shutdown" not in events["event"].values:
        print("  No shutdown events to plot.")
        return

    motor_to_joint = meta["motor_to_joint"]

    # Pair shutdown/recovered events to compute durations
    shutdown_periods = []  # (joint, start_elapsed, duration)
    active = {}  # motor_id -> start_elapsed
    for _, row in events.iterrows():
        mid = str(int(row["motor_id"]))
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        if row["event"] == "shutdown":
            active[mid] = row["elapsed_s"]
        elif row["event"] == "recovered" and mid in active:
            dur = row["elapsed_s"] - active[mid]
            shutdown_periods.append((joint, active[mid], dur))
            del active[mid]
    # Handle still-shutdown motors (never recovered)
    max_elapsed = events["elapsed_s"].max()
    for mid, start in active.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        shutdown_periods.append((joint, start, max_elapsed - start))

    if not shutdown_periods:
        print("  No completed shutdown periods to plot.")
        return

    sp_df = pd.DataFrame(shutdown_periods, columns=["joint", "start_s", "duration_s"])

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # (a) Shutdown count per joint
    counts = sp_df["joint"].value_counts().sort_index()
    colors_a = [joint_color(j) for j in counts.index]
    axes[0].barh(counts.index, counts.values, color=colors_a)
    axes[0].set_xlabel("Shutdown Count")
    axes[0].set_title("Shutdown Frequency")

    # (b) Total downtime per joint
    downtime = sp_df.groupby("joint")["duration_s"].sum().sort_index()
    colors_b = [joint_color(j) for j in downtime.index]
    axes[1].barh(downtime.index, downtime.values / 60, color=colors_b)
    axes[1].set_xlabel("Total Downtime (min)")
    axes[1].set_title("Cumulative Downtime")

    # (c) Timeline
    joints_sorted = sorted(sp_df["joint"].unique())
    joint_y = {j: i for i, j in enumerate(joints_sorted)}
    for _, row in sp_df.iterrows():
        y = joint_y[row["joint"]]
        axes[2].barh(y, row["duration_s"] / 3600, left=row["start_s"] / 3600,
                     height=0.6, color=joint_color(row["joint"]), alpha=0.7)
    axes[2].set_yticks(range(len(joints_sorted)))
    axes[2].set_yticklabels(joints_sorted)
    axes[2].set_xlabel("Time (hours)")
    axes[2].set_title("Shutdown Timeline")

    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.suptitle("Thermal Shutdown Analysis", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "shutdown_analysis.png"), dpi=150)
    plt.close(fig)
    print("  Saved shutdown_analysis.png")


# -----------------------------------------------------------------------
# Plot 3: Current over time
# -----------------------------------------------------------------------
def plot_current(df, output_dir):
    cur_df = df.dropna(subset=["current"])
    if cur_df.empty:
        print("  No current data to plot.")
        return

    fig, ax = plt.subplots(figsize=(14, 6))

    for joint in cur_df["joint_name"].unique():
        jd = cur_df[cur_df["joint_name"] == joint].sort_values("elapsed_s")
        # Downsample with rolling mean for readability
        if len(jd) > 500:
            jd = jd.set_index("elapsed_s")
            jd_resampled = jd["current"].rolling(window=20, min_periods=1).mean()
            ax.plot(jd_resampled.index / 3600, jd_resampled,
                    label=joint, color=joint_color(joint), alpha=0.7, linewidth=0.6)
        else:
            ax.plot(jd["elapsed_s"] / 3600, jd["current"],
                    label=joint, color=joint_color(joint), alpha=0.7, linewidth=0.6)

    ax.set_xlabel("Time (hours)")
    ax.set_ylabel("Current (mA)")
    ax.set_title("Motor Current Over Time")
    ax.legend(fontsize=7, ncol=4, loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "current.png"), dpi=150)
    plt.close(fig)
    print("  Saved current.png")


# -----------------------------------------------------------------------
# Plot 4: Tendon slack analysis
# -----------------------------------------------------------------------
def plot_tendon_slack(df, meta, output_dir):
    threshold = meta.get("slack_current_threshold", 30)

    # Only look at movement phases (opening / closing) where we expect current
    move_df = df[df["phase"].isin(["opening", "closing"])].dropna(subset=["current"])
    if move_df.empty:
        print("  No movement current data for slack analysis.")
        return

    joints = sorted(move_df["joint_name"].unique())

    # Bin by time window (e.g. 10-minute bins)
    move_df = move_df.copy()
    bin_minutes = 10
    move_df["time_bin"] = (move_df["elapsed_s"] / (bin_minutes * 60)).astype(int)

    # (a) Rolling average current during movement per joint
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))

    # Top: average current during movement, per time bin
    for joint in joints:
        jd = move_df[move_df["joint_name"] == joint]
        binned = jd.groupby("time_bin")["current"].mean()
        hours = binned.index * bin_minutes / 60
        axes[0].plot(hours, binned.values,
                     label=joint, color=joint_color(joint), alpha=0.8, linewidth=1)

    axes[0].set_xlabel("Time (hours)")
    axes[0].set_ylabel("Avg Current During Movement (mA)")
    axes[0].set_title("Average Current During Movement (decreasing = tendon slack)")
    axes[0].legend(fontsize=7, ncol=4, loc="upper right")
    axes[0].grid(True, alpha=0.3)

    # Bottom: slack ratio per time bin (fraction of samples below threshold)
    slack_data = {}
    for joint in joints:
        jd = move_df[move_df["joint_name"] == joint]
        is_slack = (jd["current"].abs() < threshold).astype(int)
        jd_slack = jd.copy()
        jd_slack["is_slack"] = is_slack.values
        binned = jd_slack.groupby("time_bin")["is_slack"].mean()
        slack_data[joint] = binned
        hours = binned.index * bin_minutes / 60
        axes[1].plot(hours, binned.values * 100,
                     label=joint, color=joint_color(joint), alpha=0.8, linewidth=1)

    axes[1].set_xlabel("Time (hours)")
    axes[1].set_ylabel("Slack Ratio (%)")
    axes[1].set_title(f"Tendon Slack Ratio (% of samples with |current| < {threshold} mA)")
    axes[1].legend(fontsize=7, ncol=4, loc="upper left")
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim(bottom=0)

    fig.suptitle("Tendon Slack Analysis", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "tendon_slack.png"), dpi=150)
    plt.close(fig)
    print("  Saved tendon_slack.png")

    # (b) Heatmap: joints x time bins
    if slack_data:
        all_bins = sorted(set().union(*(s.index for s in slack_data.values())))
        matrix = np.full((len(joints), len(all_bins)), np.nan)
        for i, joint in enumerate(joints):
            for j, b in enumerate(all_bins):
                if b in slack_data[joint].index:
                    matrix[i, j] = slack_data[joint][b] * 100

        fig, ax = plt.subplots(figsize=(14, 6))
        im = ax.imshow(matrix, aspect="auto", cmap="YlOrRd",
                       interpolation="nearest", vmin=0)
        ax.set_yticks(range(len(joints)))
        ax.set_yticklabels(joints, fontsize=8)

        # X-axis: show hours
        n_ticks = min(10, len(all_bins))
        tick_positions = np.linspace(0, len(all_bins) - 1, n_ticks, dtype=int)
        tick_labels = [f"{all_bins[i] * bin_minutes / 60:.1f}h" for i in tick_positions]
        ax.set_xticks(tick_positions)
        ax.set_xticklabels(tick_labels)

        ax.set_xlabel("Time")
        ax.set_title(f"Slack Ratio Heatmap (% of samples with |current| < {threshold} mA)")
        fig.colorbar(im, ax=ax, label="Slack %")
        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, "tendon_slack_heatmap.png"), dpi=150)
        plt.close(fig)
        print("  Saved tendon_slack_heatmap.png")


# -----------------------------------------------------------------------
# Summary statistics
# -----------------------------------------------------------------------
def print_summary(df, events, meta):
    print("\n" + "=" * 60)
    print("STRESS TEST SUMMARY")
    print("=" * 60)

    total_time_h = df["elapsed_s"].max() / 3600 if not df.empty else 0
    total_cycles = df["cycle"].max() if not df.empty else 0
    print(f"  Duration:       {total_time_h:.2f} hours")
    print(f"  Total cycles:   {int(total_cycles)}")

    temp_df = df.dropna(subset=["temperature"])
    if not temp_df.empty:
        print(f"\n  {'Joint':<16} {'Max°C':>6} {'Avg°C':>6}")
        print(f"  {'─' * 30}")
        for joint in sorted(temp_df["joint_name"].unique()):
            jd = temp_df[temp_df["joint_name"] == joint]
            print(f"  {joint:<16} {jd['temperature'].max():>5.1f} {jd['temperature'].mean():>5.1f}")

    # Shutdown stats
    motor_to_joint = meta["motor_to_joint"]
    if not events.empty and "shutdown" in events["event"].values:
        active = {}
        shutdown_counts = {}
        total_downtime = {}
        for _, row in events.iterrows():
            mid = str(int(row["motor_id"]))
            joint = motor_to_joint.get(mid, f"motor_{mid}")
            if row["event"] == "shutdown":
                active[mid] = row["elapsed_s"]
                shutdown_counts[joint] = shutdown_counts.get(joint, 0) + 1
            elif row["event"] == "recovered" and mid in active:
                dur = row["elapsed_s"] - active[mid]
                total_downtime[joint] = total_downtime.get(joint, 0) + dur
                del active[mid]
        # Still-active shutdowns
        max_elapsed = events["elapsed_s"].max()
        for mid, start in active.items():
            joint = motor_to_joint.get(mid, f"motor_{mid}")
            total_downtime[joint] = total_downtime.get(joint, 0) + (max_elapsed - start)

        print(f"\n  {'Joint':<16} {'Shutdowns':>10} {'Downtime':>12}")
        print(f"  {'─' * 40}")
        for joint in sorted(shutdown_counts.keys()):
            dt = total_downtime.get(joint, 0)
            mins, secs = divmod(int(dt), 60)
            print(f"  {joint:<16} {shutdown_counts[joint]:>10} {mins:>8}m {secs:>2}s")
    else:
        print("\n  No thermal shutdowns recorded.")

    # Slack summary
    threshold = meta.get("slack_current_threshold", 30)
    move_df = df[df["phase"].isin(["opening", "closing"])].dropna(subset=["current"])
    if not move_df.empty:
        print(f"\n  Tendon slack (|current| < {threshold} mA during movement):")
        print(f"  {'Joint':<16} {'Slack %':>8}")
        print(f"  {'─' * 26}")
        for joint in sorted(move_df["joint_name"].unique()):
            jd = move_df[move_df["joint_name"] == joint]
            slack_pct = (jd["current"].abs() < threshold).mean() * 100
            print(f"  {joint:<16} {slack_pct:>7.1f}%")

    print("=" * 60)


# -----------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------
def generate_plots(output_dir):
    """Generate all plots from saved stress test data."""
    df, events, meta = load_data(output_dir)

    print(f"Loaded {len(df)} readings, {len(events)} events")
    print("Generating plots...")

    plot_temperature(df, meta, output_dir)
    plot_shutdown_analysis(events, meta, output_dir)
    plot_current(df, output_dir)
    plot_tendon_slack(df, meta, output_dir)
    print_summary(df, events, meta)

    print(f"\nAll plots saved to {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description="Plot stress test results")
    parser.add_argument("output_dir", type=str,
                        help="Path to stress test output directory")
    args = parser.parse_args()

    if not os.path.isdir(args.output_dir):
        print(f"Directory not found: {args.output_dir}")
        sys.exit(1)

    generate_plots(args.output_dir)


if __name__ == "__main__":
    main()
