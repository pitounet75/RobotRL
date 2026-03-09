# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Play two-wheeler with velocity command inputs (forward speed, yaw rate).

Usage:
  # Fixed commands (e.g. go forward at 0.3 m/s, turn at 0.2 rad/s):
  isaaclab.bat -p source/isaaclab_tasks/isaaclab_tasks/two_wheeler/play_with_commands.py \\
      --task Isaac-TwoWheeler-Direct-v0 --num_envs 1 --cmd_vx 0.3 --cmd_yaw 0.2

  # Interactive keyboard control (W/S: forward/back, A/D: yaw left/right):
  isaaclab.bat -p source/isaaclab_tasks/isaaclab_tasks/two_wheeler/play_with_commands.py \\
      --task Isaac-TwoWheeler-Direct-v0 --num_envs 1 --cmd_keyboard
"""

import argparse
import os
import sys

# Add scripts/rsl_rl for cli_args import (when run from two_wheeler folder)
_script_dir = os.path.dirname(os.path.abspath(__file__))
_isaaclab_root = os.path.normpath(os.path.join(_script_dir, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(_isaaclab_root, "scripts", "reinforcement_learning", "rsl_rl"))

from isaaclab.app import AppLauncher

import cli_args  # noqa: E402

parser = argparse.ArgumentParser(description="Play two-wheeler with velocity commands.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--task", type=str, default="Isaac-TwoWheeler-Direct-v0")
parser.add_argument("--real-time", action="store_true", help="Run in real-time.")
parser.add_argument(
    "--cmd_vx",
    type=float,
    default=None,
    help="Target forward velocity [m/s]. If omitted with --cmd_keyboard, use keyboard.",
)
parser.add_argument(
    "--cmd_yaw",
    type=float,
    default=None,
    help="Target yaw rate [rad/s]. If omitted with --cmd_keyboard, use keyboard.",
)
parser.add_argument(
    "--cmd_keyboard",
    action="store_true",
    help="Use WASD for interactive control (W/S: vx, A/D: yaw).",
)
parser.add_argument("--cmd_step", type=float, default=0.1, help="Command increment for keyboard.")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)

args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import time

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab.utils.assets import retrieve_file_path

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg


def _try_keyboard_input():
    """Non-blocking keyboard read. Returns (vx_delta, yaw_delta) or (0,0)."""
    try:
        import msvcrt

        if not msvcrt.kbhit():
            return 0.0, 0.0
        ch = msvcrt.getch()
        if ch in (b"w", b"W"):
            return 1.0, 0.0
        if ch in (b"s", b"S"):
            return -1.0, 0.0
        if ch in (b"a", b"A"):
            return 0.0, 1.0
        if ch in (b"d", b"D"):
            return 0.0, -1.0
    except ImportError:
        pass
    return 0.0, 0.0


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg: RslRlBaseRunnerCfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed
    if hasattr(args_cli, "device") and args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    log_root = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root, agent_cfg.load_run, agent_cfg.load_checkpoint)

    env_cfg.log_dir = os.path.dirname(resume_path)
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    policy_nn = getattr(runner.alg, "policy", runner.alg.actor_critic)
    policy_nn.reset(torch.ones(env_cfg.scene.num_envs, dtype=torch.bool, device=env.unwrapped.device))

    dt = env.unwrapped.step_dt

    # Initial commands
    vx_cmd = args_cli.cmd_vx if args_cli.cmd_vx is not None else 0.0
    yaw_cmd = args_cli.cmd_yaw if args_cli.cmd_yaw is not None else 0.0

    if hasattr(env.unwrapped, "set_commands"):
        env.unwrapped.set_commands(vx_cmd, yaw_cmd)

    obs = env.get_observations()

    print("[Two-wheeler] Playing. Commands: vx=%.2f m/s, yaw=%.2f rad/s" % (vx_cmd, yaw_cmd))
    if args_cli.cmd_keyboard:
        print("  Keyboard: W/S = forward/back, A/D = yaw left/right (step=%.2f)" % args_cli.cmd_step)

    while simulation_app.is_running():
        start_time = time.time()

        if args_cli.cmd_keyboard and hasattr(env.unwrapped, "set_commands"):
            dv, dy = _try_keyboard_input()
            if dv != 0 or dy != 0:
                vx_cmd = max(-1.0, min(1.0, vx_cmd + dv * args_cli.cmd_step))
                yaw_cmd = max(-1.0, min(1.0, yaw_cmd + dy * args_cli.cmd_step))
                env.unwrapped.set_commands(vx_cmd, yaw_cmd)

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        if args_cli.real_time:
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
