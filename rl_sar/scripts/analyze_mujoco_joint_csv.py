#!/usr/bin/env python3
"""
MuJoCo mujoco_joint_state.csv diagnostics.

Default: compare backward, forward, and full data windows (requires the command_x column).

Pure backward segment (matches the recording convention):
  python3 analyze_mujoco_joint_csv.py data.csv --pure-backward \\
    --cmd-x-max -0.3 --cmd-y-abs-max 0.1 --cmd-yaw-abs-max 0.2 --settle-sec 3

Filter: command_x < threshold, abs(command_y) < threshold, abs(command_yaw) < threshold,
      and t >= (t of the first row satisfying those conditions) + settle_sec

Joints (d1): FL 0,1,2 | FR 3,4,5 | RL 6,7,8 | RR 9,10,11
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HIP = (0, 3, 6, 9)
FRONT_HIP = (0, 3)
REAR_HIP = (6, 9)
REAR_THIGH = (7, 10)
REAR_CALF = (8, 11)
REAR_LEG = ((6, 7, 8), (9, 10, 11))
FEET = ("FL", "FR", "RL", "RR")


def stat_series(s: pd.Series) -> dict:
    a = np.asarray(s.dropna(), dtype=np.float64)
    if a.size == 0:
        return {"n": 0, "mean": float("nan"), "std": float("nan"), "max_abs": float("nan")}
    return {
        "n": int(a.size),
        "mean": float(np.mean(a)),
        "std": float(np.std(a)),
        "max_abs": float(np.max(np.abs(a))),
    }


def fmt_stat(st: dict) -> str:
    if st["n"] == 0:
        return "n=0"
    return f"n={st['n']:<8} mean={st['mean']:+.5f} std={st['std']:.5f} max_abs={st['max_abs']:.5f}"


def mean_pool(df: pd.DataFrame, cols: list[str]) -> dict:
    stacked = []
    for c in cols:
        if c in df.columns:
            stacked.append(df[c].to_numpy(dtype=np.float64))
    if not stacked:
        return {"n": 0, "mean": float("nan"), "std": float("nan"), "max_abs": float("nan")}
    a = np.concatenate(stacked)
    a = a[~np.isnan(a)]
    return {
        "n": int(a.size),
        "mean": float(np.mean(a)),
        "std": float(np.std(a)),
        "max_abs": float(np.max(np.abs(a))),
    }


def print_foot_columns(df: pd.DataFrame) -> None:
    zcols = [f"foot_z_{leg}" for leg in FEET if f"foot_z_{leg}" in df.columns]
    if zcols:
        print("\n[foot_z world z]")
        for zname in zcols:
            zz = df[zname].to_numpy(dtype=np.float64)
            print(f"  {zname}: mean={float(np.nanmean(zz)):.5f} max={float(np.nanmax(zz)):.5f} min={float(np.nanmin(zz)):.5f}")

    ccols = [f"contact_{leg}" for leg in FEET if f"contact_{leg}" in df.columns]
    if ccols:
        print("\n[contact duty]")
        for cname in ccols:
            print(f"  {cname} mean={float(df[cname].mean()):.4f}")

    acols = [f"air_time_{leg}" for leg in FEET if f"air_time_{leg}" in df.columns]
    if acols:
        print("\n[air_time current airborne duration]")
        for aname in acols:
            aa = df[aname].to_numpy(dtype=np.float64)
            print(f"  {aname}: mean={float(np.nanmean(aa)):.5f} max={float(np.nanmax(aa)):.5f}")


def filter_pure_backward(
    df: pd.DataFrame,
    cmd_x_max: float,
    cmd_y_abs_max: float,
    cmd_yaw_abs_max: float,
    settle_sec: float,
) -> pd.DataFrame:
    need = {"command_x", "command_y", "command_yaw", "t"}
    if not need.issubset(df.columns):
        return pd.DataFrame()

    m = (
        (df["command_x"] < cmd_x_max)
        & (df["command_y"].abs() < cmd_y_abs_max)
        & (df["command_yaw"].abs() < cmd_yaw_abs_max)
    )
    sub = df.loc[m].copy()
    if len(sub) == 0:
        return sub
    t_enter = float(sub["t"].min())
    sub = sub[sub["t"] >= t_enter + settle_sec]
    return sub


def print_joint_groups(df: pd.DataFrame, title: str) -> None:
    print(f"\n======== {title} ========")
    if len(df) == 0:
        print("No data.")
        return

    print(f"Valid rows: {len(df)}")

    def row_joint(label: str, idx: int) -> None:
        qcol, aq = f"q{idx}", f"arm_q{idx}"
        acol = f"action{idx}"
        if qcol not in df.columns:
            return
        print(f"\n--- [{label}] index {idx} ---")
        print(f"  q       {fmt_stat(stat_series(df[qcol]))}")
        if aq in df.columns:
            print(f"  arm_q   {fmt_stat(stat_series(df[aq]))}")
        if acol in df.columns:
            print(f"  action  {fmt_stat(stat_series(df[acol]))}")

    print("\n[Rear hip]")
    for idx in REAR_HIP:
        row_joint("rear_hip", idx)

    print("\n[Rear thigh]")
    for idx in REAR_THIGH:
        row_joint("rear_thigh", idx)

    print("\n[Rear calf]")
    for idx in REAR_CALF:
        row_joint("rear_calf", idx)

    # Optional: pose / feet / contact
    opt_pitch = next((c for c in ("base_pitch", "pitch") if c in df.columns), None)
    opt_roll = next((c for c in ("base_roll", "roll") if c in df.columns), None)
    if opt_pitch or opt_roll:
        print("\n[Base orientation rad]")
        if opt_roll:
            print(f"  base_roll  {fmt_stat(stat_series(df[opt_roll]))}")
        if opt_pitch:
            print(f"  base_pitch {fmt_stat(stat_series(df[opt_pitch]))}")

    print_foot_columns(df)

    rh_q = [f"q{i}" for i in REAR_HIP]
    rh_arm = [f"arm_q{i}" for i in REAR_HIP]
    print("\n--- rear hip pooled ---")
    print(f"  q       {fmt_stat(mean_pool(df, rh_q))}")
    print(f"  arm_q   {fmt_stat(mean_pool(df, rh_arm))}")

    print("\n======== Interpretation ========")
    print(
        "If rear hip max_abs in the pure segment is still around ~0.2-0.3 rad but the rear legs look more jumpy,"
        "the cause is more likely thigh/calf swing timing, flight time, or mixed-in turning/lateral motion, rather than an isolated hip issue."
    )


def print_compare_mode(df: pd.DataFrame) -> None:
    has_cmd = "command_x" in df.columns
    backward = pd.DataFrame()
    forward = pd.DataFrame()
    if not has_cmd:
        print(
            "\n[Warning] command_x column is missing: cannot filter backward/forward windows.\n"
            "The following joint statistics use the full dataset (for reference only)."
        )
        sections: list[tuple[str, pd.DataFrame]] = [("[All data]", df)]
    else:
        backward = df[df["command_x"] < -0.3].copy()
        forward = df[df["command_x"] > 0.3].copy()
        print(f"\ncommand_x < -0.3 (backward): {len(backward)} rows")
        print(f"command_x > +0.3 (forward): {len(forward)} rows")
        sections = [
            ("[Backward window]", backward),
            ("[Forward window]", forward),
            ("[All data]", df),
        ]

    HIP_ALL = (0, 3, 6, 9)
    for label, sub in sections:
        if len(sub) == 0:
            print(f"\n{label} has no data; skipping.")
            continue
        print(f"\n======== {label} ========")
        print("\n--- hip q / arm_q ---")
        for i in HIP_ALL:
            qc, ac = f"q{i}", f"arm_q{i}"
            if qc not in sub.columns:
                continue
            print(f"  q{i:2d}    {fmt_stat(stat_series(sub[qc]))}")
            if ac in sub.columns:
                print(f"  arm_q{i} {fmt_stat(stat_series(sub[ac]))}")

        fh = [f"q{i}" for i in FRONT_HIP]
        rh = [f"q{i}" for i in REAR_HIP]
        print("\n--- front vs rear hip (q pooled) ---")
        print(f"  front hip q:  {fmt_stat(mean_pool(sub, fh))}")
        print(f"  rear hip q:   {fmt_stat(mean_pool(sub, rh))}")
        afh = [f"arm_q{i}" for i in FRONT_HIP]
        arh = [f"arm_q{i}" for i in REAR_HIP]
        print(f"  front arm_q:  {fmt_stat(mean_pool(sub, afh))}")
        print(f"  rear arm_q:   {fmt_stat(mean_pool(sub, arh))}")

        print("\n--- rear legs RL 6,7,8 | RR 9,10,11 (pooled q / arm_q) ---")
        for leg_name, triple in ("RL", REAR_LEG[0]), ("RR", REAR_LEG[1]):
            cols_q = [f"q{i}" for i in triple]
            cols_a = [f"arm_q{i}" for i in triple]
            print(f"  {leg_name} q:    {fmt_stat(mean_pool(sub, cols_q))}")
            print(f"  {leg_name} arm_q:{fmt_stat(mean_pool(sub, cols_a))}")

        print_foot_columns(sub)

    if has_cmd and len(backward) > 0 and len(forward) > 0:
        print("\n======== Interpretation Notes ========")
        r_b = mean_pool(backward, [f"arm_q{i}" for i in REAR_HIP])["max_abs"]
        r_f = mean_pool(forward, [f"arm_q{i}" for i in REAR_HIP])["max_abs"]
        q_b = mean_pool(backward, [f"q{i}" for i in REAR_HIP])["max_abs"]
        q_f = mean_pool(forward, [f"q{i}" for i in REAR_HIP])["max_abs"]
        print(
            "During backward motion, a large rear arm_q means target angles or the policy dominate;"
            "small arm_q with large q means PD, contact, or friction tracking error."
        )
        print(f"  backward rear arm_q max_abs(pooled): {r_b:.5f}  | forward: {r_f:.5f}")
        print(f"  backward rear q    max_abs(pooled): {q_b:.5f}  | forward: {q_f:.5f}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyze MuJoCo mujoco_joint_state.csv")
    ap.add_argument("csv", type=Path, nargs="?", default=Path("mujoco_joint_state.csv"))
    ap.add_argument(
        "--pure-backward",
        action="store_true",
        help="Only output pure backward segment statistics (command_x/x/yaw + settle filtering)",
    )
    ap.add_argument("--cmd-x-max", type=float, default=-0.3, help="command_x upper bound (more negative means backward)")
    ap.add_argument("--cmd-y-abs-max", type=float, default=0.1)
    ap.add_argument("--cmd-yaw-abs-max", type=float, default=0.2)
    ap.add_argument(
        "--settle-sec",
        type=float,
        default=3.0,
        help="After entering a matching command segment, discard the first settle seconds (approximate RL settling before statistics)",
    )
    args = ap.parse_args()
    if not args.csv.is_file():
        print(f"File not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(args.csv)
    print(f"File: {args.csv}")
    print(f"Rows: {len(df)}  Columns: {len(df.columns)}")

    if args.pure_backward:
        sub = filter_pure_backward(
            df,
            args.cmd_x_max,
            args.cmd_y_abs_max,
            args.cmd_yaw_abs_max,
            args.settle_sec,
        )
        print("\nPure backward filter parameters:")
        print(f"  command_x < {args.cmd_x_max}")
        print(f"  abs(command_y) < {args.cmd_y_abs_max}")
        print(f"  abs(command_yaw) < {args.cmd_yaw_abs_max}")
        print(f"  settle_sec = {args.settle_sec} (relative to the first t that satisfies the conditions above)")
        if len(sub) == 0:
            print("\n[Error] No data after filtering. Check whether the CSV contains command_* columns, or relax thresholds / shorten settle.")
            sys.exit(2)
        print_joint_groups(sub, "Pure backward window (settled segment)")
        sys.exit(0)

    print_compare_mode(df)


if __name__ == "__main__":
    main()
