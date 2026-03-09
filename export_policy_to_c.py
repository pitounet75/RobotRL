#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""
Export trained two-wheeler policy to C code for microcontroller deployment.

Usage:
    python export_policy_to_c.py --checkpoint logs/rsl_rl/two_wheeler/<run>/model_400.pt
    python export_policy_to_c.py --checkpoint path/to/model.pt --output_dir ./embedded

Generates:
    two_wheeler_policy.h - Header with weights and inference declaration
    two_wheeler_policy.c - C implementation for microcontroller
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Export two-wheeler policy to C")
    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to the RSL-RL checkpoint (.pt file)",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=None,
        help="Output directory (default: same as script)",
    )
    parser.add_argument(
        "--action_scale",
        type=float,
        default=3.0,
        help="Scale factor for actions (default: 3.0 Nm)",
    )
    return parser.parse_args()


def extract_actor_weights(checkpoint_path: str) -> dict:
    """Load checkpoint and extract actor weights."""
    import torch

    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if "model_state_dict" in ckpt:
        state_dict = ckpt["model_state_dict"]
    else:
        state_dict = ckpt

    # Find actor layers (MLP: linear layers)
    actor_weights = {}
    for key, value in state_dict.items():
        if "actor" in key and "actor_obs_normalizer" not in key:
            actor_weights[key] = value.numpy()

    return actor_weights


def elu_c(x: str) -> str:
    """Generate ELU activation in C."""
    return f"(({x}) > 0.0f ? ({x}) : (expf({x}) - 1.0f))"


def generate_c_code(weights: dict, action_scale: float) -> tuple[str, str]:
    """Generate C header and source files."""

    # Extract weight matrices - RSL-RL actor MLP structure
    # actor.0 = Linear(6, 64), actor.1 = ELU, actor.2 = Linear(64, 64), actor.3 = ELU, actor.4 = Linear(64, 2)
    w0 = weights["actor.0.weight"]  # (64, 6)
    b0 = weights["actor.0.bias"]    # (64,)
    w1 = weights["actor.2.weight"]  # (64, 64)
    b1 = weights["actor.2.bias"]    # (64,)
    w2 = weights["actor.4.weight"]  # (2, 64)
    b2 = weights["actor.4.bias"]    # (2,)

    IN_DIM = 6
    HIDDEN_DIM = 64
    OUT_DIM = 2

    def format_array(arr, indent="    "):
        lines = []
        flat = arr.flatten()
        for i in range(0, len(flat), 8):
            chunk = flat[i : i + 8]
            vals = ", ".join(f"{v:.8e}f" for v in chunk)
            lines.append(indent + vals + ",")
        return "\n".join(lines)

    header = f'''/**
 * Two-wheeler balancing policy - C header for microcontroller deployment
 * Network: 6 -> 64 (ELU) -> 64 (ELU) -> 2
 *
 * Observation order: [pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate]
 * Output: [left_torque, right_torque] in range [-1, 1], scale by {action_scale}f for Nm
 */

#ifndef TWO_WHEELER_POLICY_H
#define TWO_WHEELER_POLICY_H

#ifdef __cplusplus
extern "C" {{
#endif

#define POLICY_IN_DIM   {IN_DIM}
#define POLICY_HIDDEN   {HIDDEN_DIM}
#define POLICY_OUT_DIM {OUT_DIM}

/**
 * Run policy inference.
 *
 * @param obs  Input observations [pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate]
 * @param out  Output torques [left_Nm, right_Nm], already scaled (max ±3 Nm)
 */
void two_wheeler_policy_inference(const float* obs, float* out);

#ifdef __cplusplus
}}
#endif

#endif /* TWO_WHEELER_POLICY_H */
'''

    source = f'''/**
 * Two-wheeler balancing policy - C implementation
 * Requires: #include <math.h> for expf()
 */

#include "two_wheeler_policy.h"
#include <math.h>

#define ACTION_SCALE {action_scale}f

/* Layer 0: Linear(6, 64) */
static const float W0[{HIDDEN_DIM}][{IN_DIM}] = {{
{format_array(w0)}
}};
static const float B0[{HIDDEN_DIM}] = {{
{format_array(b0.reshape(-1, 1))}
}};

/* Layer 1: Linear(64, 64) */
static const float W1[{HIDDEN_DIM}][{HIDDEN_DIM}] = {{
{format_array(w1)}
}};
static const float B1[{HIDDEN_DIM}] = {{
{format_array(b1.reshape(-1, 1))}
}};

/* Layer 2: Linear(64, 2) */
static const float W2[{OUT_DIM}][{HIDDEN_DIM}] = {{
{format_array(w2)}
}};
static const float B2[{OUT_DIM}] = {{
{format_array(b2.reshape(-1, 1))}
}};

/* ELU activation */
static inline float elu(float x) {{
    return (x > 0.0f) ? x : (expf(x) - 1.0f);
}}

void two_wheeler_policy_inference(const float* obs, float* out) {{
    float h0[{HIDDEN_DIM}];
    float h1[{HIDDEN_DIM}];

    /* Layer 0: Linear + ELU */
    for (int i = 0; i < {HIDDEN_DIM}; i++) {{
        float sum = B0[i];
        for (int j = 0; j < {IN_DIM}; j++) {{
            sum += W0[i][j] * obs[j];
        }}
        h0[i] = elu(sum);
    }}

    /* Layer 1: Linear + ELU */
    for (int i = 0; i < {HIDDEN_DIM}; i++) {{
        float sum = B1[i];
        for (int j = 0; j < {HIDDEN_DIM}; j++) {{
            sum += W1[i][j] * h0[j];
        }}
        h1[i] = elu(sum);
    }}

    /* Layer 2: Linear (output) */
    for (int i = 0; i < {OUT_DIM}; i++) {{
        float sum = B2[i];
        for (int j = 0; j < {HIDDEN_DIM}; j++) {{
            sum += W2[i][j] * h1[j];
        }}
        /* Clamp to [-1, 1] and scale to torque */
        if (sum > 1.0f) sum = 1.0f;
        if (sum < -1.0f) sum = -1.0f;
        out[i] = sum * ACTION_SCALE;
    }}
}}
'''

    return header, source


def main():
    args = parse_args()

    if not os.path.exists(args.checkpoint):
        print(f"Error: Checkpoint not found: {args.checkpoint}")
        return 1

    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).parent
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading checkpoint: {args.checkpoint}")
    weights = extract_actor_weights(args.checkpoint)

    # Verify we have the expected layers
    expected = ["actor.0.weight", "actor.0.bias", "actor.2.weight", "actor.2.bias", "actor.4.weight", "actor.4.bias"]
    for k in expected:
        if k not in weights:
            print(f"Error: Expected key '{k}' not found in checkpoint.")
            print(f"Available keys: {list(weights.keys())}")
            return 1

    print("Generating C code...")
    header, source = generate_c_code(weights, args.action_scale)

    header_path = output_dir / "two_wheeler_policy.h"
    source_path = output_dir / "two_wheeler_policy.c"

    with open(header_path, "w") as f:
        f.write(header)
    with open(source_path, "w") as f:
        f.write(source)

    print(f"Generated: {header_path}")
    print(f"Generated: {source_path}")
    print("\nObservation order: [pitch, pitch_rate, v_x, w_rw, w_lw, yaw_rate]")
    print("Output: [left_torque_Nm, right_torque_Nm] (already scaled)")
    return 0


if __name__ == "__main__":
    exit(main())
