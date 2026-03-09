# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Run the two-wheeler environment with PID balancing control (no RL policy)."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Run two-wheeler with PID balancing controller."
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=2,
    help="Number of environments to simulate.",
)
parser.add_argument(
    "--task",
    type=str,
    default="Isaac-TwoWheeler-Direct-v0",
    help="Name of the task.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg


def main():
    """Run two-wheeler with PID controller."""
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=getattr(args_cli, "device", "cpu"),
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    env_cfg.sim.device = "cpu"
    env_cfg.use_pid = True

    env = gym.make(args_cli.task, cfg=env_cfg)

    print("[INFO]: Running two-wheeler with PID controller")
    print(f"[INFO]: Simulation device: {env_cfg.sim.device}")
    print(f"[INFO]: PID gains Kp={env_cfg.pid_Kp}, Ki={env_cfg.pid_Ki}, Kd={env_cfg.pid_Kd}")

    env.reset()

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = torch.zeros(
                env.action_space.shape, device=env.unwrapped.device
            )
            env.step(actions)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
