---
title: Week 8 - Isaac Sim Basics
---

# Week 8: Isaac Sim Basics

## Learning Objectives

By the end of this week, you will be able to:

- Understand the NVIDIA Isaac Sim platform and its capabilities.
- Launch Isaac Sim and navigate its interface.
- Import and manipulate 3D robot models (USD).
- Perform basic simulations and interact with the environment.

## Core Concepts

### NVIDIA Isaac Sim

NVIDIA Isaac Sim is a scalable robotics simulation application built on NVIDIA Omniverse. It provides a platform for:

-   **High-Fidelity Simulation**: Realistic physics, rendering, and sensor simulation.
-   **Scalability**: Run multiple simulations concurrently for large-scale training.
-   **Omniverse Integration**: Leverage Universal Scene Description (USD) for collaborative 3D workflows.
-   **ROS/ROS 2 Support**: Seamless integration with ROS and ROS 2 for robot control and data exchange.

### USD (Universal Scene Description)

USD is a powerful framework for describing, composing, simulating, and collaborating on 3D scenes. Isaac Sim heavily relies on USD for its scene representation, allowing for modularity and extensibility.

### Isaac Sim Interface

Isaac Sim provides a comprehensive GUI (Graphical User Interface) for scene manipulation, simulation control, and debugging. Key components include:

-   **Stage**: The main 3D viewport where your scene is displayed.
-   **Layer Editor**: Manages USD layers for scene composition.
-   **Property Window**: Edits properties of selected objects.
-   **Content Browser**: Accesses assets (models, materials, environments).

## Hands-On Lab

### Lab 8.1: Exploring Isaac Sim and Importing a Robot

**Objective**: Launch Isaac Sim, navigate its interface, and import a pre-existing robot model.

**Prerequisites**:

-   NVIDIA GPU (RTX series recommended for optimal performance).
-   Docker (for running Isaac Sim container) or native installation.
-   NVIDIA Container Toolkit installed.

**Steps**:

1.  **Launch Isaac Sim (using Docker)**:
    
    ```bash
    # Make sure to have NVIDIA Container Toolkit installed
    # Pull the latest Isaac Sim image (check NVIDIA NGC for the correct tag)
    docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

    # Run Isaac Sim (adjust paths as needed)
    docker run --name isaac-sim --privileged --gpus all -e "ACCEPT_EULA=Y" --network host \
        -v ~/isaac-sim/cache/ov:/root/.cache/ov:rw \
        -v ~/isaac-sim/cache/pip:/root/.cache/pip:rw \
        -v ~/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
        -v ~/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
        -v ~/isaac-sim/logs:/root/.nvidia-omniverse/logs/Kit/IsaacSim:rw \
        -v ~/isaac-sim/data:/root/.local/share/ov/data:rw \
        -v ~/isaac-sim/documents:/root/Documents:rw \
        -it nvcr.io/nvidia/isaac-sim:2023.1.1
    ```
    *Note: The first launch might take a while to download assets.*

2.  **Navigate the Isaac Sim Interface**:
    -   Once Isaac Sim loads, familiarize yourself with the Stage, Layer Editor, and Property Window.
    -   Use `Alt + Left Click` to orbit, `Alt + Right Click` to zoom, `Alt + Middle Click` to pan.

3.  **Import a Robot Model**:
    -   Go to `Window > Asset Browser`.
    -   Navigate to `Assets > NVIDIA > Isaac > Robots`.
    -   Drag and drop a robot model (e.g., `franka/franka_alt_fingers.usd`) into the Stage.

4.  **Perform a Basic Simulation**:
    -   Press the "Play" button (▶️) in the toolbar to start the physics simulation.
    -   Observe the robot falling due to gravity (if not fixed in place).
    -   Try moving the robot around manually in the editor and then playing the simulation again.

### Expected Output

You will successfully launch Isaac Sim, be able to navigate the 3D environment, and import a robot model that responds to physics simulation when you press play.

## Checkpoint Quiz

<details>
<summary>Question 1: What is the core technology that Isaac Sim is built upon for 3D scene description?</summary>

NVIDIA Isaac Sim is built on NVIDIA Omniverse, which uses Universal Scene Description (USD) as its core technology for describing and composing 3D scenes.

</details>

<details>
<summary>Question 2: Name two benefits of using Isaac Sim for robotics development.</summary>

Benefits include high-fidelity simulation, scalability for large-scale training, Omniverse integration for collaborative workflows, and strong ROS/ROS 2 support for robot control.

</details>

## References & Further Reading

- NVIDIA Isaac Sim Documentation: [https://docs.omniverse.nvidia.com/isaacsim/latest/index.html](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- Universal Scene Description (USD): [https://graphics.pixar.com/usd/docs/index.html](https://graphics.pixar.com/usd/docs/index.html)
- Isaac Sim Tutorials: [https://docs.omniverse.nvidia.com/isaacsim/latest/setup/tutorials.html](https://docs.omniverse.nvidia.com/isaacsim/latest/setup/tutorials.html)

## Diagrams

```mermaid
graph TD
    USD[USD Assets (Robots, Environments)] --> IsaacSim[NVIDIA Isaac Sim]
    IsaacSim --> Physics[Physics Engine]
    IsaacSim --> Rendering[High-Fidelity Rendering]
    IsaacSim -- ROS/ROS 2 Integration --> RoboticsCode[Robot Control Code]
```
