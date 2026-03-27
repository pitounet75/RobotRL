# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Run multiple trainings in sequence, sweeping a config parameter.

After each training, appends results to a status file so you can compare which
parameter value performed best.

Usage:
  cmd /c "call conda.bat activate H:\isaac_env_v5 && cd /d H:\Projects\IsaacV5\IsaacLab && python source/isaaclab_tasks/isaaclab_tasks/two_wheeler/sweep_param.py --param rew_pitch_rate_w --min 0.1 --max 1.9 --step 0.01"

  # Shorter runs per value (e.g. 500 iterations):
  ... --max_iterations 500

Results file: H:\Projects\IsaacV5\IsaacLab\logs\rsl_rl\two_wheeler\sweep_results.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
from datetime import datetime

# Add two_wheeler folder for config_overrides
_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)
import config_overrides  # noqa: E402

# All overridable parameter names (env rew_* only; train_with_overrides uses skip_rsl=True)
VALID_PARAMS = [n for n, _ in config_overrides.ENV_REW_OVERRIDES]

LOG_FILE = os.path.join(
    os.path.normpath(os.path.join(_script_dir, "..", "..", "..", "..")),
    "logs", "rsl_rl", "two_wheeler", "training_output.log",
)
RESULTS_FILE = os.path.join(
    os.path.normpath(os.path.join(_script_dir, "..", "..", "..", "..")),
    "logs", "rsl_rl", "two_wheeler", "sweep_results.csv",
)


def _generate_values(min_val: float, max_val: float, step: float) -> list[float]:
    """Generate values from min to max (inclusive) by step, handling float precision."""
    values = []
    v = min_val
    while v <= max_val + 1e-9:
        values.append(round(v, 10))
        v += step
    return values


def _parse_last_metrics(log_path: str) -> dict | None:
    """Parse the last Learning iteration block from the training log."""
    if not os.path.isfile(log_path):
        return None
    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    # Find last occurrence of each metric
    patterns = {
        "mean_reward": r"Mean reward:\s*([-\d.]+)",
        "mean_episode_length": r"Mean episode length:\s*([-\d.]+)",
        "mean_action_noise_std": r"Mean action noise std:\s*([-\d.]+)",
        "iteration": r"Learning iteration\s+(\d+)/\d+",
    }
    result = {}
    for key, pat in patterns.items():
        matches = list(re.finditer(pat, content))
        if matches:
            result[key] = matches[-1].group(1)
    return result if len(result) == 4 else None


def main():
    parser = argparse.ArgumentParser(
        description="Sweep a config parameter across multiple training runs."
    )
    parser.add_argument(
        "--param",
        type=str,
        required=True,
        choices=VALID_PARAMS,
        help="Parameter to sweep (e.g. rew_pitch_rate_w).",
    )
    parser.add_argument("--min", type=float, required=True, help="Minimum value (inclusive).")
    parser.add_argument("--max", type=float, required=True, help="Maximum value (inclusive).")
    parser.add_argument("--step", type=float, required=True, help="Step between values.")
    parser.add_argument(
        "--max_iterations",
        type=int,
        default=None,
        help="Max iterations per run (default: config default).",
    )
    parser.add_argument(
        "--num_envs",
        type=int,
        default=2048,
        help="Number of environments (default: 2048).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Device (default: cuda:0).",
    )
    args = parser.parse_args()

    if args.min > args.max:
        print("[SWEEP] Error: --min must be <= --max")
        sys.exit(1)
    if args.step <= 0:
        print("[SWEEP] Error: --step must be positive")
        sys.exit(1)

    values = _generate_values(args.min, args.max, args.step)
    print(f"[SWEEP] Parameter: {args.param}")
    print(f"[SWEEP] Values: {len(values)} runs from {args.min} to {args.max} by {args.step}")
    print(f"[SWEEP] Results file: {RESULTS_FILE}")

    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    file_exists = os.path.isfile(RESULTS_FILE)
    fieldnames = [
        "param", "param_value", "mean_reward", "mean_episode_length",
        "mean_action_noise_std", "final_iteration", "run_name", "exit_code",
        "timestamp",
    ]

    train_script = "source/isaaclab_tasks/isaaclab_tasks/two_wheeler/train_with_overrides.py"

    def build_cmd(value: float) -> str:
        run_name = f"sweep_{args.param}_{value}".replace(".", "_")
        extra = f" --{args.param} {value} --run_name {run_name}"
        if args.max_iterations is not None:
            extra += f" --max_iterations {args.max_iterations}"
        return (
            'cmd /c "call conda.bat activate H:\\isaac_env_v5 && '
            'cd /d H:\\Projects\\IsaacV5\\IsaacLab && '
            f'isaaclab.bat -p {train_script} '
            '--task Isaac-TwoWheeler-Direct-v0 --headless '
            f'--num_envs {args.num_envs} --device {args.device}'
            f'{extra} '
            '> logs\\rsl_rl\\two_wheeler\\training_output.log 2>&1"'
        )

    with open(RESULTS_FILE, "a", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()

        for i, value in enumerate(values):
            run_name = f"sweep_{args.param}_{value}".replace(".", "_")
            print(f"\n[SWEEP] Run {i+1}/{len(values)}: {args.param}={value}")

            cmd = build_cmd(value)
            exit_code = subprocess.run(cmd, shell=True).returncode

            metrics = _parse_last_metrics(LOG_FILE)
            row = {
                "param": args.param,
                "param_value": value,
                "mean_reward": metrics.get("mean_reward", "") if metrics else "",
                "mean_episode_length": metrics.get("mean_episode_length", "") if metrics else "",
                "mean_action_noise_std": metrics.get("mean_action_noise_std", "") if metrics else "",
                "final_iteration": metrics.get("iteration", "") if metrics else "",
                "run_name": run_name,
                "exit_code": exit_code,
                "timestamp": datetime.now().isoformat(),
            }
            writer.writerow(row)
            csvfile.flush()

            if metrics:
                print(f"  Mean reward: {row['mean_reward']}, Episode length: {row['mean_episode_length']}")
            else:
                print(f"  (Could not parse metrics from log)")

    print(f"\n[SWEEP] Done. Results saved to: {RESULTS_FILE}")
    print("To find the best value: sort the CSV by mean_reward (descending).")


if __name__ == "__main__":
    main()
