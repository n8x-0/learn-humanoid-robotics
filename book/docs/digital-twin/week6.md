---
title: Week 6 - Gazebo Simulation
---

# Week 6: Gazebo Simulation

## Learning Objectives

By the end of this week, you will be able to:

- Understand the purpose and architecture of Gazebo.
- Launch Gazebo environments and load robot models.
- Control simulated robots using ROS 2.
- Create simple Gazebo worlds and models.

## Core Concepts

### What is Gazebo?

Gazebo is a powerful 3D robot simulator that allows you to accurately and efficiently test robotics algorithms in a realistic virtual environment. It provides:

-   **Physics Engine**: Simulates gravity, inertia, friction, and collisions.
-   **High-Quality Graphics**: Renders realistic environments and robot models.
-   **Sensor Simulation**: Simulates various sensors like cameras, LiDAR, IMU.
-   **ROS 2 Integration**: Seamless integration with ROS 2 for robot control and data exchange.

### Gazebo Architecture

Gazebo consists of:

-   **Server (`gzserver`)**: The physics engine and simulation core.
-   **Client (`gzclient`)**: A graphical user interface (GUI) for visualizing the simulation.
-   **Worlds**: XML files defining the environment (terrain, objects, lighting).
-   **Models**: XML files defining robots and other objects (links, joints, sensors, actuators).

### SDF (Simulation Description Format)

SDF is the XML format used by Gazebo to describe robots and environments. It's similar to URDF (Unified Robot Description Format) but supports more features for simulation.

## Hands-On Lab

### Lab 6.1: Simulating a Simple Robot in Gazebo

**Objective**: Launch Gazebo, load a basic robot model, and control it using ROS 2.

**Prerequisites**:

-   ROS 2 Humble/Iron installed (from Week 3)
-   Gazebo installed:
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs # Or ros-iron-gazebo-ros-pkgs
    ```

**Steps**:

1.  **Create a new ROS 2 package for your Gazebo simulation** (e.g., `my_robot_description`).
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake my_robot_description
    cd my_robot_description
    ```

2.  **Define a simple URDF model** (e.g., a differential drive robot) in `urdf/my_robot.urdf`.
    
    ```xml
    <!-- urdf/my_robot.urdf -->
    <?xml version="1.0"?>
    <robot name="my_diff_drive_robot">
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.1" radius="0.2"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 1 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder length="0.1" radius="0.2"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
      </link>
    </robot>
    ```

3.  **Create a Gazebo world file** (e.g., `worlds/empty.world`).
    
    ```xml
    <!-- worlds/empty.world -->
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="empty">
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>
      </world>
    </sdf>
    ```

4.  **Create a launch file** (`launch/robot_sim.launch.py`) to spawn the robot in Gazebo.
    
    ```python
    # launch/robot_sim.launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        pkg_my_robot_description = get_package_share_directory('my_robot_description')

        # Start Gazebo server and client
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': os.path.join(pkg_my_robot_description, 'worlds', 'empty.world')}.items(),
        )

        # Spawn robot
        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-file', os.path.join(pkg_my_robot_description, 'urdf', 'my_robot.urdf'),
                                       '-entity', 'my_diff_drive_robot'],
                            output='screen')

        return LaunchDescription([
            gazebo,
            spawn_entity
        ])
    ```

5.  **Update `CMakeLists.txt`** in `my_robot_description/CMakeLists.txt`:

    ```cmake
    # ...
    find_package(ament_cmake REQUIRED)
    find_package(gazebo_ros REQUIRED)
    find_package(urdf REQUIRED)

    install(DIRECTORY urdf
      DESTINATION share/${PROJECT_NAME}
    )
    install(DIRECTORY worlds
      DESTINATION share/${PROJECT_NAME}
    )
    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
    )

    ament_package()
    ```

6.  **Update `package.xml`**:

    ```xml
    <!-- my_robot_description/package.xml -->
    <depend>gazebo_ros</depend>
    <depend>urdf</depend>
    <depend>xacro</depend> <!-- Often useful for complex URDFs -->
    <exec_depend>gazebo</exec_depend>
    ```

7.  **Build and source your workspace** (from `~/ros2_ws`):

    ```bash
    colcon build --packages-select my_robot_description
    source install/setup.bash
    ```

8.  **Launch the simulation**:

    ```bash
    ros2 launch my_robot_description robot_sim.launch.py
    ```

### Expected Output

Gazebo will launch, and you should see a blue cylindrical robot model in the empty world. You can move around the view in Gazebo to inspect the robot.

## Checkpoint Quiz

<details>
<summary>Question 1: What are the two main components of Gazebo and their functions?</summary>

`gzserver` is the physics engine and simulation core, running headless. `gzclient` is the graphical user interface for visualizing the simulation and interacting with it.

</details>

<details>
<summary>Question 2: What is the primary file format used by Gazebo to describe environments and robot models?</summary>

SDF (Simulation Description Format).

</details>

## References & Further Reading

- Gazebo Documentation: [http://gazebosim.org/tutorials](http://gazebosim.org/tutorials)
- ROS 2 Gazebo Integration: [https://navigation.ros.org/setup_guides/simulation/setup_simulation.html](https://navigation.ros.org/setup_guides/simulation/setup_simulation.html)
- URDF to SDF conversion: [http://classic.gazebosim.org/tutorials?tut=ros_urdf](http://classic.gazebosim.org/tutorials?tut=ros_urdf)

## Diagrams

```mermaid
graph TD
    A[Launch File] --> B[Gazebo Server (gzserver)]
    A --> C[Gazebo Client (gzclient)]
    A --> D[Spawn Entity Node]
    D --> B
    B -- Physics & Sensors --> C
```
