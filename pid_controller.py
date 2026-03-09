# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Vectorized PID controller for Isaac Lab environments."""

from __future__ import annotations

import torch


class PIDController:
    """Vectorized PID controller for Isaac Lab environments.

    Implements: u = Kp * error + Ki * integral(error) + Kd * d(error)/dt
    where error = target - current.
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        dt: float,
        num_envs: int,
        num_dof: int,
        device: str | torch.device,
        integral_limit: float = 1.0,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral = torch.zeros(num_envs, num_dof, device=device)
        self._prev_error = torch.zeros(num_envs, num_dof, device=device)

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """Reset integrator and previous error for specified environments."""
        if env_ids is None:
            self.integral.zero_()
            self._prev_error.zero_()
        else:
            self.integral[env_ids] = 0.0
            self._prev_error[env_ids] = 0.0

    def compute(self, target: torch.Tensor, current: torch.Tensor) -> torch.Tensor:
        """Compute PID output. Error = target - current.

        Args:
            target: Target values. Shape (num_envs, num_dof).
            current: Current values. Shape (num_envs, num_dof).

        Returns:
            Control output. Shape (num_envs, num_dof).
        """
        error = target - current
        self.integral += error * self.dt
        self.integral = torch.clamp(
            self.integral, -self.integral_limit, self.integral_limit
        )
        derivative = (error - self._prev_error) / self.dt
        self._prev_error.copy_(error)
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
