# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Two-wheel self-balancing robot environment."""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.assets import Articulation
from isaaclab.sim.spawners.from_files import spawn_ground_plane
from isaaclab.utils.math import euler_xyz_from_quat as quat_to_euler_xyz, sample_uniform

from .pid_controller import PIDController
from .two_wheeler_env_cfg import TwoWheelerEnvCfg


class TwoWheelerEnv(DirectRLEnv):
    """Balance a two-wheel robot with wheel torques (Direct workflow)."""

    cfg: TwoWheelerEnvCfg

    def __init__(self, cfg: TwoWheelerEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]

        self._default_root_pos_w = self.robot.data.root_pos_w.clone().expand(
            self.num_envs, -1
        )
        self._default_root_quat_w = self.robot.data.root_quat_w.clone().expand(
            self.num_envs, -1
        )
        self._default_root_lin_vel_w = self.robot.data.root_lin_vel_w.clone().expand(
            self.num_envs, -1
        )
        self._default_root_ang_vel_w = self.robot.data.root_ang_vel_w.clone().expand(
            self.num_envs, -1
        )
        self._default_joint_pos = self.robot.data.joint_pos.clone().expand(
            self.num_envs, -1
        )
        self._default_joint_vel = self.robot.data.joint_vel.clone().expand(
            self.num_envs, -1
        )

        self._rw_idx = self.robot.find_joints(self.cfg.right_wheel_dof_name)[0][0]
        self._lw_idx = self.robot.find_joints(self.cfg.left_wheel_dof_name)[0][0]
        self._wheel_ids = torch.tensor(
            [self._rw_idx, self._lw_idx], device=self.device, dtype=torch.long
        )

        self.actions = torch.zeros(
            (self.num_envs, self.cfg.action_space), device=self.device
        )
        self.prev_actions = torch.zeros_like(self.actions)
        self.torque_cmd = torch.zeros(
            (self.num_envs, self.cfg.action_space), device=self.device
        )
        self._settle_steps = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.long
        )

        # Velocity commands: [v_x_cmd, yaw_rate_cmd] in body frame
        self.commands = torch.zeros(
            (self.num_envs, 2), device=self.device, dtype=torch.float32
        )

        if getattr(self.cfg, "use_pid", False):
            self.pid = PIDController(
                Kp=self.cfg.pid_Kp,
                Ki=self.cfg.pid_Ki,
                Kd=self.cfg.pid_Kd,
                dt=self.step_dt,
                num_envs=self.num_envs,
                num_dof=1,
                device=self.device,
                integral_limit=self.cfg.pid_integral_limit,
            )

        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel

    def _setup_scene(self) -> None:
        robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=self.cfg.ground_cfg)
        self.scene.clone_environments(copy_from_source=False)

        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["robot"] = robot

        light_cfg = sim_utils.DomeLightCfg(
            intensity=2000.0, color=(0.75, 0.75, 0.75)
        )
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if getattr(self.cfg, "use_pid", False):
            pitch, _ = self._get_pitch_and_rate()
            target_pitch = torch.zeros_like(pitch)
            u = self.pid.compute(target_pitch, -pitch)
            u = u.squeeze(-1)
            self.torque_cmd = torch.stack([u, u], dim=-1) * self.cfg.pid_torque_scale
            self.torque_cmd = torch.clamp(
                self.torque_cmd, -self.cfg.max_torque, self.cfg.max_torque
            )
        else:
            self.prev_actions[:] = self.actions
            self.actions = torch.clamp(actions, -1.0, 1.0)
            self.torque_cmd = self.cfg.action_scale * self.actions

        # If yaw command is ~0, keep wheel torques symmetric to prevent spin.
        if getattr(self.cfg, "enforce_symmetric_torque", False):
            yaw_cmd = self.commands[:, 1]
            no_yaw = torch.abs(yaw_cmd) < getattr(self.cfg, "yaw_hold_epsilon", 1e-3)
            if no_yaw.any():
                mean_torque = self.torque_cmd[no_yaw].mean(dim=-1, keepdim=True)
                self.torque_cmd[no_yaw] = mean_torque.repeat(1, 2)

        # Hold zero torque for a few steps after reset to avoid bounce/flip.
        settle_mask = self._settle_steps > 0
        if settle_mask.any():
            self._settle_steps[settle_mask] -= 1
            self.torque_cmd[settle_mask] = 0.0
            self.actions[settle_mask] = 0.0
            self.prev_actions[settle_mask] = 0.0

    def _get_pitch_and_rate(self) -> tuple[torch.Tensor, torch.Tensor]:
        quat_w = self.robot.data.root_quat_w
        _, pitch, _ = quat_to_euler_xyz(quat_w)
        pitch_rate = self.robot.data.root_ang_vel_w[:, 1]
        return pitch.unsqueeze(-1), pitch_rate.unsqueeze(-1)

    def _apply_action(self) -> None:
        self.robot.set_joint_effort_target(self.torque_cmd, joint_ids=self._wheel_ids)

    def step(self, action: torch.Tensor):
        """Override to accumulate reward at each physics substep."""
        if not getattr(self.cfg, "rew_accumulate_substeps", False):
            return super().step(action)

        action = action.to(self.device)
        if self.cfg.action_noise_model:
            action = self._action_noise_model(action)

        self._pre_physics_step(action)

        is_rendering = self.sim.has_gui() or self.sim.has_rtx_sensors()
        reward_acc = torch.zeros(self.num_envs, device=self.device, dtype=torch.float32)

        for _ in range(self.cfg.decimation):
            self._sim_step_counter += 1
            self._apply_action()
            self.scene.write_data_to_sim()
            self.sim.step(render=False)
            if self._sim_step_counter % self.cfg.sim.render_interval == 0 and is_rendering:
                self.sim.render()
            self.scene.update(dt=self.physics_dt)
            reward_acc += self._get_rewards()

        self.episode_length_buf += 1
        self.common_step_counter += 1

        self.reset_terminated[:], self.reset_time_outs[:] = self._get_dones()
        self.reset_buf = self.reset_terminated | self.reset_time_outs
        # Average over substeps to keep reward scale similar to single-step
        self.reward_buf = reward_acc / self.cfg.decimation

        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            self._reset_idx(reset_env_ids)
            if self.sim.has_rtx_sensors() and self.cfg.num_rerenders_on_reset > 0:
                for _ in range(self.cfg.num_rerenders_on_reset):
                    self.sim.render()

        if self.cfg.events and "interval" in self.event_manager.available_modes:
            self.event_manager.apply(mode="interval", dt=self.step_dt)

        self.obs_buf = self._get_observations()
        if self.cfg.observation_noise_model:
            self.obs_buf["policy"] = self._observation_noise_model(self.obs_buf["policy"])

        return self.obs_buf, self.reward_buf, self.reset_terminated, self.reset_time_outs, self.extras

    def _get_observations(self) -> dict:
        quat_w = self.robot.data.root_quat_w
        _, pitch, _ = quat_to_euler_xyz(quat_w)
        ang_vel_w = self.robot.data.root_ang_vel_w
        lin_vel_b = self.robot.data.root_com_lin_vel_b

        w_rw = self.joint_vel[:, self._rw_idx].flatten()
        w_lw = self.joint_vel[:, self._lw_idx].flatten()
        pitch_rate = ang_vel_w[:, 1]
        yaw_rate = ang_vel_w[:, 2]
        v_x = lin_vel_b[:, 0]

        # obs: pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate, v_x_cmd, yaw_rate_cmd
        obs = torch.cat(
            [
                torch.stack(
                    [pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate], dim=-1
                ),
                self.commands,
            ],
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        quat_w = self.robot.data.root_quat_w
        _, pitch, _ = quat_to_euler_xyz(quat_w)
        ang_vel_w = self.robot.data.root_ang_vel_w
        pitch_rate = ang_vel_w[:, 1]
        yaw_rate = ang_vel_w[:, 2]
        lin_vel_b = self.robot.data.root_com_lin_vel_b
        v_x = lin_vel_b[:, 0]

        r_upright = torch.exp(-self.cfg.rew_upright_k * pitch**2)
        r_pitch_rate = -self.cfg.rew_pitch_rate_w * pitch_rate**2
        # Track commanded velocity and yaw rate (exp reward)
        v_x_err = (v_x - self.commands[:, 0]) ** 2
        yaw_err = (yaw_rate - self.commands[:, 1]) ** 2
        r_forward = (
            self.cfg.rew_track_forward_w
            * torch.exp(-v_x_err / self.cfg.rew_track_vel_scale)
        )
        r_yaw_rate = (
            self.cfg.rew_track_yaw_w
            * torch.exp(-yaw_err / self.cfg.rew_track_yaw_scale)
        )
        r_torque = -self.cfg.rew_torque_w * torch.sum(self.torque_cmd**2, dim=-1)
        r_action_rate = -self.cfg.rew_action_rate_w * torch.sum(
            (self.actions - self.prev_actions) ** 2, dim=-1
        )

        return (
            r_upright + r_pitch_rate + r_yaw_rate + r_forward + r_torque + r_action_rate
        )

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        quat_w = self.robot.data.root_quat_w
        _, pitch, _ = quat_to_euler_xyz(quat_w)
        done_fall = torch.abs(pitch) > self.cfg.terminate_pitch_rad
        done_timeout = self.episode_length_buf >= self.max_episode_length
        return done_fall, done_timeout

    def set_commands(
        self,
        v_x_cmd: float | torch.Tensor,
        yaw_rate_cmd: float | torch.Tensor,
        env_ids: torch.Tensor | None = None,
    ) -> None:
        """Set velocity commands for external control (e.g. during play).

        Args:
            v_x_cmd: Target forward velocity [m/s] in body frame.
            yaw_rate_cmd: Target yaw rate [rad/s].
            env_ids: Env indices to update. If None, updates all envs.
        """
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if isinstance(v_x_cmd, (int, float)):
            v_x_cmd = torch.full(
                (len(env_ids),), float(v_x_cmd), device=self.device, dtype=torch.float32
            )
        if isinstance(yaw_rate_cmd, (int, float)):
            yaw_rate_cmd = torch.full(
                (len(env_ids),),
                float(yaw_rate_cmd),
                device=self.device,
                dtype=torch.float32,
            )
        self.commands[env_ids, 0] = v_x_cmd
        self.commands[env_ids, 1] = yaw_rate_cmd

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = env_ids.long().view(-1)

        # Reset base buffers (including episode_length_buf)
        super()._reset_idx(env_ids)

        self.robot.reset(env_ids)

        pitch0 = sample_uniform(
            self.cfg.initial_pitch_range[0],
            self.cfg.initial_pitch_range[1],
            (len(env_ids),),
            device=self.device,
        )
        half = 0.5 * pitch0
        q_w = torch.cos(half)
        q_x = torch.zeros_like(q_w)
        q_y = torch.sin(half)
        q_z = torch.zeros_like(q_w)
        quat = torch.stack([q_w, q_x, q_y, q_z], dim=-1)

        root_state = torch.zeros((len(env_ids), 13), device=self.device)
        root_pos = self._default_root_pos_w[env_ids] + self.scene.env_origins[env_ids]
        root_pos[:, 2] += getattr(
            self.cfg, "init_root_height_offset", 0.0
        )  # lower so wheels touch ground
        root_state[:, 0:3] = root_pos
        root_state[:, 3:7] = quat
        root_state[:, 7:10] = self._default_root_lin_vel_w[env_ids]
        root_state[:, 10:13] = self._default_root_ang_vel_w[env_ids]
        self.robot.write_root_state_to_sim(root_state, env_ids)

        w0 = sample_uniform(
            self.cfg.initial_wheel_vel_range[0],
            self.cfg.initial_wheel_vel_range[1],
            (len(env_ids), 2),
            device=self.device,
        )
        joint_pos = self._default_joint_pos[env_ids].clone()
        joint_vel = self._default_joint_vel[env_ids].clone()
        joint_vel[:, self._rw_idx] = w0[:, 0]
        joint_vel[:, self._lw_idx] = w0[:, 1]
        joint_ids = torch.tensor(
            [self._rw_idx, self._lw_idx], device=self.device, dtype=torch.long
        )
        self.robot.write_joint_state_to_sim(
            joint_pos, joint_vel, joint_ids=joint_ids, env_ids=env_ids
        )

        self.actions[env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self._settle_steps[env_ids] = getattr(self.cfg, "reset_settle_steps", 0)

        # Sample new velocity commands
        v_x_cmd = sample_uniform(
            self.cfg.command_vel_x_range[0],
            self.cfg.command_vel_x_range[1],
            (len(env_ids),),
            device=self.device,
        )
        yaw_cmd = sample_uniform(
            self.cfg.command_yaw_rate_range[0],
            self.cfg.command_yaw_rate_range[1],
            (len(env_ids),),
            device=self.device,
        )
        self.commands[env_ids, 0] = v_x_cmd
        self.commands[env_ids, 1] = yaw_cmd

        if getattr(self.cfg, "use_pid", False):
            self.pid.reset(env_ids)
