# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Two-wheel self-balancing robot environment."""

import gymnasium as gym

from .two_wheeler_env import TwoWheelerEnv
from .two_wheeler_env_cfg import TwoWheelerEnvCfg

gym.register(
    id="Isaac-TwoWheeler-Direct-v0",
    entry_point=f"{__name__}.two_wheeler_env:TwoWheelerEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.two_wheeler_env_cfg:TwoWheelerEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_cfg:TwoWheelerPPORunnerCfg",
    },
)

gym.register(
    id="TwoWheelerEnv",
    entry_point=f"{__name__}.two_wheeler_env:TwoWheelerEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.two_wheeler_env_cfg:TwoWheelerEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_cfg:TwoWheelerPPORunnerCfg",
    },
)
