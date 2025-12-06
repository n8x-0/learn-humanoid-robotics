---
title: NVIDIA Isaac Platform - Introduction
---

# NVIDIA Isaac Platform (Weeks 8–10)

## Overview

Explore NVIDIA's Isaac platform for advanced robotics.

## Learning Path

- [Week 8: Isaac Sim Basics](/docs/isaac/week8)
- [Week 9: Isaac GYM](/docs/isaac/week9)
- [Week 10: Advanced Isaac](/docs/isaac/week10)

## Diagrams

```mermaid
graph TD
    IsaacSim[Isaac Sim (High-Fidelity Simulation)]
    IsaacGym[Isaac Gym (RL Acceleration)]
    Omniverse[NVIDIA Omniverse (USD Framework)]

    Omniverse -- powers --> IsaacSim
    Omniverse -- powers --> IsaacGym

    IsaacSim -- ROS/ROS 2 Integration --> RobotControl[Robot Control & Perception]
    IsaacGym -- RL Training --> AgentPolicy[Agent Policy]

    subgraph NVIDIA Isaac Platform
        Omniverse & IsaacSim & IsaacGym
    end
```
