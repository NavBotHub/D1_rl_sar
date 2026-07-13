#!/usr/bin/env python3
"""
Compare two mujoco_joint_state.csv files under the same pure backward + settle rule (old policy vs new policy).

Usage (run from the rl_sar directory or provide absolute paths):
  python3 scripts/compare_mujoco_pure_backward.py \\
    logs/baseline.csv logs/model6000.csv \\
    --settle-sec 3

Prerequisite: both CSV files must be recorded by the same rl_sim_mujoco version (with command_* columns; foot_z_* / contact_* are recommended).

model_6000.pt: first convert it to ONNX with scripts/convert_policy.py and overwrite policy/d1/robot_lab/policy_onnx.onnx,
then collect pure backward data with the fixed procedure to produce new_csv.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

_scripts = Path(__file__).resolve().parent
if str(_scripts) not in sys.path:
    sys.path.insert(0, str(_scripts))

import analyze_mujoco_joint_csv as ac  # noqa: E402


def pure_metrics(df: pd.DataFrame) -> dict[str, float]:
    """Key scalars on the pure backward subset (aligned with the team KPI convention)."""
    m: dict[str, float] = {"n_rows": float(len(df))}
    if len(df) == 0:
        for k in (
            "foot_z_RL_max",
            "foot_z_RR_max",
            "contact_RL_mean",
            "contact_RR_mean",
            "rear_hip_q_max_abs",
            "rear_thigh_q_max_abs",
            "rear_calf_q_max_abs",
        ):
            m[k] = float("nan")
        return m

    for label, col in (
        ("foot_z_RL_max", "foot_z_RL"),
        ("foot_z_RR_max", "foot_z_RR"),
    ):
        if col in df.columns:
            m[label] = float(np.nanmax(df[col].to_numpy(dtype=np.float64)))
        else:
            m[label] = float("nan")

    for label, col in (
        ("contact_RL_mean", "contact_RL"),
        ("contact_RR_mean", "contact_RR"),
    ):
        if col in df.columns:
            m[label] = float(df[col].mean())
        else:
            m[label] = float("nan")

    rh = [f"q{i}" for i in ac.REAR_HIP]
    rt = [f"q{i}" for i in ac.REAR_THIGH]
    rc = [f"q{i}" for i in ac.REAR_CALF]
    m["rear_hip_q_max_abs"] = ac.mean_pool(df, rh)["max_abs"]
    m["rear_thigh_q_max_abs"] = ac.mean_pool(df, rt)["max_abs"]
    m["rear_calf_q_max_abs"] = ac.mean_pool(df, rc)["max_abs"]
    return m


def fmt(v: float, nd: int = 5) -> str:
    if v != v:  # NaN
        return "N/A"
    return f"{v:.{nd}f}"


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare pure backward segment KPIs for two CSV files")
    ap.add_argument("baseline_csv", type=Path, help="Baseline (old) CSV")
    ap.add_argument("new_csv", type=Path, help="New policy CSV")
    ap.add_argument("--cmd-x-max", type=float, default=-0.3)
    ap.add_argument("--cmd-y-abs-max", type=float, default=0.1)
    ap.add_argument("--cmd-yaw-abs-max", type=float, default=0.2)
    ap.add_argument("--settle-sec", type=float, default=3.0)
    args = ap.parse_args()

    for p in (args.baseline_csv, args.new_csv):
        if not p.is_file():
            print(f"File not found: {p}", file=sys.stderr)
            sys.exit(1)

    df0 = pd.read_csv(args.baseline_csv)
    df1 = pd.read_csv(args.new_csv)

    sub0 = ac.filter_pure_backward(
        df0,
        args.cmd_x_max,
        args.cmd_y_abs_max,
        args.cmd_yaw_abs_max,
        args.settle_sec,
    )
    sub1 = ac.filter_pure_backward(
        df1,
        args.cmd_x_max,
        args.cmd_y_abs_max,
        args.cmd_yaw_abs_max,
        args.settle_sec,
    )

    m0 = pure_metrics(sub0)
    m1 = pure_metrics(sub1)

    print("Pure backward filter:", f"cmd_x<{args.cmd_x_max}", f"|y|<{args.cmd_y_abs_max}", f"|yaw|<{args.cmd_yaw_abs_max}")
    print(f"settle_sec={args.settle_sec}(relative to the first t that satisfies the condition)\n")

    print(f"{'Metric':<26} {'Baseline(old)':>14} {'New(model6000...)':>18} {'Delta(new-old)':>12}")
    print("-" * 72)
    keys = [
        ("Valid rows", "n_rows", 0),
        ("foot_z_RL max (m)", "foot_z_RL_max", 5),
        ("foot_z_RR max (m)", "foot_z_RR_max", 5),
        ("contact_RL mean", "contact_RL_mean", 4),
        ("contact_RR mean", "contact_RR_mean", 4),
        ("rear hip |q| max", "rear_hip_q_max_abs", 5),
        ("rear thigh |q| max", "rear_thigh_q_max_abs", 5),
        ("rear calf |q| max", "rear_calf_q_max_abs", 5),
    ]
    for title, key, nd in keys:
        a, b = m0[key], m1[key]
        d = b - a if (a == a and b == b) else float("nan")
        print(
            f"{title:<26} {fmt(a, nd):>14} {fmt(b, nd):>18} {fmt(d, nd):>12}"
        )

    print("\n-- Target reference (team working guideline, for quick review) --")
    print("  foot_z max: expect RL/RR to drop from about ~0.10/~0.09 to ~0.05~0.07")
    print("  contact duty: expect it to rise from ~0.33 to ~0.40~0.50")
    print("  rear thigh/calf max_abs: expect them to decrease\n")

    print("-- If the new CSV changes relative to the baseline suggest --")
    fz0 = (m0["foot_z_RL_max"] + m0["foot_z_RR_max"]) / 2
    fz1 = (m1["foot_z_RL_max"] + m1["foot_z_RR_max"]) / 2
    cd0 = (m0["contact_RL_mean"] + m0["contact_RR_mean"]) / 2
    cd1 = (m1["contact_RL_mean"] + m1["contact_RR_mean"]) / 2
    if fz1 == fz1 and fz0 == fz0 and fz1 < fz0 - 0.005:
        if cd1 == cd1 and cd0 == cd0 and abs(cd1 - cd0) < 0.03:
            print(
                "- foot_z decreases significantly but contact duty barely changes -> next, set feet_air_time "
                "from 2.0 to 1.0 or 1.5 in training, then train another version for comparison."
            )
    if fz1 == fz1 and fz1 > 0.085:
        print(
            "- foot_z is still around ~0.09~0.10 -> strengthen the foot_clearance penalty in training (for example -0.01 -> -0.02),"
            "or lower the clearance target further (for example -0.36); also watch whether velocity/tracking gets worse."
        )
    print(
        "- If lin_vel / command tracking gets significantly worse: do not keep forcing foot height lower; add clearer gait/duty "
        "constraints instead of only changing clearance.\n"
    )


if __name__ == "__main__":
    main()
