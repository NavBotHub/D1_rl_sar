#!/usr/bin/env python3
"""
在同一条「纯后退 + settle」规则下，对比两条 mujoco_joint_state.csv（旧策略 vs 新策略）。

用法（在 rl_sar 目录或给出绝对路径）:
  python3 scripts/compare_mujoco_pure_backward.py \\
    logs/baseline.csv logs/model6000.csv \\
    --settle-sec 3

前置：两条 CSV 均由同一版 rl_sim_mujoco 录制（含 command_*；建议含 foot_z_* / contact_*）。

model_6000.pt：先用 scripts/convert_policy.py 转为 ONNX，覆盖 policy/d1/robot_lab/policy_onnx.onnx，
再按固定流程采纯后退数据得到 new_csv。
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
    """纯后退子集上的关键标量（与团队约定的 KPI 对齐）。"""
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
        return "—"
    return f"{v:.{nd}f}"


def main() -> None:
    ap = argparse.ArgumentParser(description="对比两条 CSV 的纯后退段 KPI")
    ap.add_argument("baseline_csv", type=Path, help="基准（旧）CSV")
    ap.add_argument("new_csv", type=Path, help="新策略 CSV")
    ap.add_argument("--cmd-x-max", type=float, default=-0.3)
    ap.add_argument("--cmd-y-abs-max", type=float, default=0.1)
    ap.add_argument("--cmd-yaw-abs-max", type=float, default=0.2)
    ap.add_argument("--settle-sec", type=float, default=3.0)
    args = ap.parse_args()

    for p in (args.baseline_csv, args.new_csv):
        if not p.is_file():
            print(f"找不到文件: {p}", file=sys.stderr)
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

    print("纯后退筛选:", f"cmd_x<{args.cmd_x_max}", f"|y|<{args.cmd_y_abs_max}", f"|yaw|<{args.cmd_yaw_abs_max}")
    print(f"settle_sec={args.settle_sec}（相对首次满足条件的 t）\n")

    print(f"{'指标':<26} {'基准(旧)':>14} {'新(model6000…)':>18} {'Δ(新-旧)':>12}")
    print("-" * 72)
    keys = [
        ("有效行数", "n_rows", 0),
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

    print("\n—— 目标参考（团队口头约定，便于扫一眼）——")
    print("  foot_z max：希望 RL/RR 约从 ~0.10/~0.09 降到 ~0.05~0.07")
    print("  contact duty：希望从 ~0.33 升到 ~0.40~0.50")
    print("  rear thigh/calf max_abs：希望下降\n")

    print("—— 若新 CSV 相对基准的变化 suggests ——")
    fz0 = (m0["foot_z_RL_max"] + m0["foot_z_RR_max"]) / 2
    fz1 = (m1["foot_z_RL_max"] + m1["foot_z_RR_max"]) / 2
    cd0 = (m0["contact_RL_mean"] + m0["contact_RR_mean"]) / 2
    cd1 = (m1["contact_RL_mean"] + m1["contact_RR_mean"]) / 2
    if fz1 == fz1 and fz0 == fz0 and fz1 < fz0 - 0.005:
        if cd1 == cd1 and cd0 == cd0 and abs(cd1 - cd0) < 0.03:
            print(
                "· foot_z 明显下降，但 contact duty 几乎没变 → 下一步可在训练里把 feet_air_time "
                "从 2.0 调到 1.0 或 1.5，再训一版对比。"
            )
    if fz1 == fz1 and fz1 > 0.085:
        print(
            "· foot_z 仍在 ~0.09~0.10 → 可在训练侧加强 foot_clearance 惩罚（如 -0.01→-0.02），"
            "或把 clearance 目标再压低（如 -0.36）；同时盯 velocity/tracking 是否变差。"
        )
    print(
        "· 若 lin_vel / command tracking 明显变差：不宜继续硬压脚高；应加更明确的 gait/duty "
        "约束，而不是只改 clearance。\n"
    )


if __name__ == "__main__":
    main()
