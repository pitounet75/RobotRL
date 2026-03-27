# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL PPO configuration for the two-wheeler task."""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class TwoWheelerPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO runner configuration for two-wheeler balancing."""

    experiment_name = "two_wheeler"
    run_name = "ppo"
    max_iterations = 4000
    save_interval = 200

    num_steps_per_env = 16
    num_learning_epochs = 4
    num_mini_batches = 4

    policy = RslRlPpoActorCriticCfg(
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="elu",
        init_noise_std=1,
    )

    algorithm = RslRlPpoAlgorithmCfg(
        learning_rate=3e-4,
        clip_param=0.2,
        entropy_coef=0.0004,
        gamma=0.99,
        lam=0.95,
        max_grad_norm=1.0,
        value_loss_coef=1.0,
        num_learning_epochs=4,
        num_mini_batches=4,
        schedule="adaptive",
        desired_kl=0.01,
        use_clipped_value_loss=True,
    )
