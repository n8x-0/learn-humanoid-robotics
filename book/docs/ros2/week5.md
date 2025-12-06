---
title: Week 5 - Advanced ROS 2 Concepts
---

# Week 5: Advanced ROS 2 Concepts

## Learning Objectives

By the end of this week, you will be able to:

- Understand the concept of ROS 2 launch files for managing multiple nodes.
- Utilize ROS 2 parameters to configure node behavior dynamically.
- Implement basic debugging techniques for ROS 2 applications.
- Explore the concept of TF (Transform) for coordinate frame management.

## Core Concepts

### ROS 2 Launch Files

Launch files are XML or Python scripts used to start and manage multiple ROS 2 nodes and their configurations (parameters, remappings) simultaneously. They simplify complex system bring-up.

```xml
<!-- example.launch.xml -->
<launch>
    <node pkg="my_ros2_package" exec="publisher" name="my_publisher" output="screen">
        <param name="frequency" value="2.0"/>
    </node>
    <node pkg="my_ros2_package" exec="subscriber" name="my_subscriber" output="screen"/>
</launch>
```

### ROS 2 Parameters

Parameters allow you to configure nodes at runtime without recompiling. Nodes declare parameters, and values can be set via launch files, the command line, or `ros2 param` commands.

**Declaring a parameter in a node (Python)**:
```python
# In your Node's __init__
self.declare_parameter('frequency', 1.0) # default value
self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
```

**Setting parameters via command line**:
```bash
ros2 run my_ros2_package publisher --ros-args -p frequency:=5.0
```

### ROS 2 TF (Transformations)

TF is a system for keeping track of multiple coordinate frames and transforming data between them. It's crucial for robotics, where various sensors and effectors have their own local coordinate systems.

- **Static Transforms**: Fixed relationships between frames (e.g., base_link to camera_link).
- **Dynamic Transforms**: Changing relationships (e.g., world to robot base).
- **`tf2_ros`**: ROS 2 package for TF functionalities.

## Hands-On Lab

### Lab 5.1: Using Launch Files and Parameters

**Objective**: Create a launch file to start your publisher and subscriber nodes, and configure the publisher's frequency using a parameter.

**Steps**:

1.  **Modify `simple_publisher.py`** to declare and use a `frequency` parameter:

    ```python
    # my_ros2_package/my_ros2_package/simple_publisher.py (changes in __init__ and timer_callback)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher')
            self.declare_parameter('frequency', 1.0) # Declare parameter with default 1.0 Hz
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            # Get parameter value and set up timer
            timer_period = 1.0 / self.get_parameter('frequency').get_parameter_value().double_value
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello ROS 2: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1
    # ... (rest of the file remains the same)
    ```

2.  **Create a Python launch file (`launch/my_nodes.launch.py`)** in your `my_ros2_package` directory.

    ```python
    # my_ros2_package/launch/my_nodes.launch.py
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_ros2_package',
                executable='publisher',
                name='my_publisher',
                output='screen',
                parameters=[
                    {'frequency': 5.0} # Override default frequency to 5.0 Hz
                ]
            ),
            Node(
                package='my_ros2_package',
                executable='subscriber',
                name='my_subscriber',
                output='screen',
            )
        ])
    ```

3.  **Update `setup.py`** to install the launch file (add `data_files` entry):

    ```python
    # my_ros2_package/setup.py
    # ...
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))), # Add this line
    ],
    # ...
    ```
    *Don't forget to import `glob` and `os`*: `from glob import glob`, `import os`.

4.  **Build and source your workspace** (from `~/ros2_ws`):

    ```bash
    colcon build --packages-select my_ros2_package
    source install/setup.bash
    ```

5.  **Run the launch file**:

    ```bash
    ros2 launch my_ros2_package my_nodes.launch.py
    ```

### Expected Output

You will see both publisher and subscriber nodes start. The publisher should be sending messages at approximately 5 Hz (much faster than the default 1 Hz from Week 3), and the subscriber should be receiving them at the same rate.

## Checkpoint Quiz

<details>
<summary>Question 1: What is the benefit of using ROS 2 launch files?</summary>

Launch files allow you to define and manage the startup of multiple ROS 2 nodes, set their parameters, and manage remappings from a single configuration file, simplifying complex system deployment and configuration.

</details>

<details>
<summary>Question 2: How can you dynamically change a node's behavior without recompiling it?</summary>

By using ROS 2 parameters. Nodes declare parameters, which can then be set and modified at runtime via launch files, command-line arguments, or specific `ros2 param` commands.

</details>

## References & Further Reading

- ROS 2 Launch Files: [https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Launch-Files/Using-A-Launch-File.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Launch-Files/Using-A-Launch-File.html)
- ROS 2 Parameters: [https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- ROS 2 TF2: [https://docs.ros.org/en/foxy/Tutorials/Learning-ROS2-TF2.html](https://docs.ros.org/en/foxy/Tutorials/Learning-ROS2-TF2.html)

## Diagrams

```mermaid
graph TD
    L[Launch File] --> P[Publisher Node(frequency=5.0)]
    L --> S[Subscriber Node]
    P -- "chatter topic" --> S
```
