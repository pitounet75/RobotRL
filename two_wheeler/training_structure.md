# Two-Wheeler RL Training Structure

> **To render in mermaid.live:** Copy ONLY the content between the backticks (not including the ` ```mermaid ` line). Paste into https://mermaid.live

## Hierarchical Overview

```mermaid
flowchart TB
    subgraph OneIteration["One Iteration"]
        Coll[Collection Phase - Rollout]
        Learn[Learning Phase - PPO Update]
        Coll --> Learn
    end
    I1[Iteration 1] --> I2[Iteration 2] --> I3[Iteration 3]
```

## Iteration Detail: Collection Phase

```mermaid
flowchart LR
    S1[RL Step 1] --> S2[RL Step 2] --> S3[RL Step 3] --> S4["..."] --> S16[RL Step 16]
```

## Episode vs Rollout (Single Environment)

```mermaid
flowchart LR
    A1[Step 1] --> A2[Step 2] --> A3[Step 3] --> A4[Step 4] --> A5[Step 5]
    A5 -->|FALL RESET| B1
    B1[Step 6] --> B2[Step 7] --> B3[Step 8] --> B4[Step 9] --> B5[Step 10]
    B5 -->|FALL RESET| C1
    C1[Step 11] --> C2[Step 12] --> C3[...]
```

## Hierarchy Summary

```mermaid
flowchart TB
    T[Training]
    T --> I[Iteration]
    I --> C[Collection - RL steps]
    I --> L[Learning - PPO update]
    C --> R[RL Step]
    R --> E[num_envs Environments]
    R --> P[decimation Physics Steps]
    E --> EP[Episode - reset to done]
    P --> S[sim.step]
```

## Key Quantities

| Term | Meaning |
|------|---------|
| **Iteration** | One collect + learn cycle |
| **RL Step** | One policy decision per env |
| **Episode** | One trajectory: reset until termination or timeout |
| **Physics Step** | One `sim.step(dt)` |
| **step_dt** | `sim.dt × decimation` (time per RL step) |

---

*View this file in VS Code with a Mermaid extension, or paste the code blocks into [mermaid.live](https://mermaid.live) to render.*
