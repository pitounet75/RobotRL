# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the two-wheeler balancing environment."""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg
from isaaclab.utils import configclass


@configclass
class TwoWheelerEnvCfg(DirectRLEnvCfg):
    """Direct workflow RL task: balance a two-wheel robot upright with wheel torques."""

    # ------------------------
    # RL / env
    # ------------------------
    decimation = 1
    episode_length_s = 10.0

    action_space = 2
    observation_space = 8  # pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate, v_x_cmd, yaw_rate_cmd
    state_space = 0

    max_torque = 3.0
    action_scale = max_torque

    # ------------------------
    # Simulation
    # ------------------------
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        device="cpu",
    )

    # ------------------------
    # Scene
    # ------------------------
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=2,
        env_spacing=3.0,
        replicate_physics=True,
        clone_in_fabric=True,
    )

    # ------------------------
    # Robot asset
    # ------------------------
    usd_path: str = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "two_wheeler", "two_wheeler.usda"
    )

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
        ),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["RWheelRevoluteJoint", "LWheelRevoluteJoint"],
                effort_limit=max_torque,
                velocity_limit=200.0,
                stiffness=0.0,
                damping=0.0,
            )
        },
    )

    right_wheel_dof_name: str = "RWheelRevoluteJoint"
    left_wheel_dof_name: str = "LWheelRevoluteJoint"

    # ------------------------
    # Ground
    # ------------------------
    ground_cfg: GroundPlaneCfg = GroundPlaneCfg()

    # ------------------------
    # Reset / termination
    # ------------------------
    terminate_pitch_rad = 0.9
    initial_pitch_range = (-0.3, 0.3)    # [rad] initial pitch perturbation
    initial_wheel_vel_range = (0.0, 0.0)   # no initial spin: avoids drop/bounce
    init_root_height_offset = -0.02        # lower robot so wheel bottoms touch ground (tune if needed)
    reset_settle_steps = 10                # hold zero torque after reset to avoid bounce/flip

    # ------------------------
    # Velocity commands (targets for forward speed, yaw rate)
    # ------------------------
    command_vel_x_range = (0, 0)       # [m/s] forward velocity target
    command_yaw_rate_range = (-0, 0)    # [rad/s] yaw rate target
    yaw_hold_epsilon = 1e-3            # treat |yaw_cmd| < eps as "no turn"
    enforce_symmetric_torque = False    # force equal wheel torque when yaw_cmd ≈ 0

    # ------------------------
    # Reward weights
    # ------------------------
    rew_upright_k = 10.0
    rew_pitch_rate_w = 0.3   # penalize fast pitch changes (smoother balancing)
    rew_track_vel_scale = 0.25   # exp(-v_x_err²/scale) for velocity tracking
    rew_track_yaw_scale = 0.25   # exp(-yaw_err²/scale) for yaw tracking
    rew_track_forward_w = 0.5    # weight for forward velocity tracking reward
    rew_track_yaw_w = 2        # weight for yaw rate tracking reward
    rew_torque_w = 0.01       # stronger torque penalty reduces jerky corrections
    rew_action_rate_w = 0.05   # penalize abrupt action changes (key for smooth control)
    rew_terminated = -3.0
    rew_accumulate_substeps = True  # accumulate reward at each physics substep (smoother penalty for jerkiness)

    # ------------------------
    # PID controller (when use_pid=True)
    # ------------------------
    use_pid: bool = False
    pid_Kp: float = 0.15
    pid_Ki: float = 0.08
    pid_Kd: float = 0.01
    pid_integral_limit: float = 1.0
    pid_torque_scale: float = 3.0
