---
title: Week 7 - Unity Integration
---

# Week 7: Unity Integration

## Learning Objectives

By the end of this week, you will be able to:

- Understand the benefits of using Unity for digital twin development.
- Integrate ROS 2 with Unity using the ROS-Unity Bridge.
- Create a basic robot simulation in Unity and control it via ROS 2.
- Leverage Unity's rendering capabilities for realistic visualizations.

## Core Concepts

### Unity for Robotics Simulation

Unity is a popular real-time 3D development platform. For robotics, it offers:

-   **High-Fidelity Rendering**: Create visually rich and realistic environments.
-   **Rich Asset Store**: Access a vast library of 3D models, textures, and tools.
-   **Extensibility**: Develop custom plugins and tools using C# scripting.
-   **Human-Robot Interaction (HRI)**: Ideal for simulating complex HRI scenarios.

### ROS-Unity Bridge

The ROS-Unity Bridge (or Unity Robotics Hub) provides a communication layer between ROS 2 and Unity. It enables:

-   **ROS 2 Message Exchange**: Send and receive ROS 2 topics, services, and actions.
-   **Robot Model Import**: Import URDF/SDF models into Unity.
-   **Sensor Simulation**: Simulate Unity cameras, LiDAR, and other sensors as ROS 2 topics.

## Hands-On Lab

### Lab 7.1: Connecting Unity with ROS 2

**Objective**: Set up a Unity project, integrate the ROS-Unity Bridge, and control a simple Unity robot from a ROS 2 node.

**Prerequisites**:

-   Unity Hub and Unity Editor installed (latest LTS version recommended).
-   ROS 2 Humble/Iron installed (from Week 3).
-   A basic ROS 2 publisher node (from Week 3 or 4) that publishes a `Twist` message for robot movement.

**Steps**:

1.  **Create a new Unity 3D Project**.
    -   Open Unity Hub, click "New Project", select "3D Core", and name it `UnityRos2Robot`.

2.  **Install Unity Robotics ROS 2 packages**.
    -   In Unity Editor, go to `Window > Package Manager`.
    -   Click the `+` icon -> `Add package from git URL...`.
    -   Add `com.unity.robotics.ros-tcp-connector` and `com.unity.robotics.ros2-msgs`.

3.  **Set up a simple robot in Unity**.
    -   Create a new 3D Object (e.g., a Cube) in your scene.
    -   Add a `Rigidbody` component to it (`Add Component > Physics > Rigidbody`).
    -   Create a C# script (e.g., `RobotController.cs`) and attach it to the Cube.

4.  **Implement `RobotController.cs`** to receive ROS 2 `Twist` messages and move the Cube.

    ```csharp
    // Assets/Scripts/RobotController.cs
    using UnityEngine;
    using RosMessageTypes.Geometry;
    using Unity.Robotics.ROSTCPConnector;

    public class RobotController : MonoBehaviour
    {
        public float linearSpeed = 1.0f;
        public float angularSpeed = 50.0f;
        private Rigidbody rb;

        void Start()
        {
            rb = GetComponent<Rigidbody>();
            ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("cmd_vel", TwistCallback);
        }

        void TwistCallback(TwistMsg twistMessage)
        {
            // Apply linear velocity
            Vector3 linearVel = new Vector3((float)twistMessage.linear.x, 0, (float)twistMessage.linear.y) * linearSpeed;
            rb.velocity = transform.TransformDirection(linearVel);

            // Apply angular velocity
            float angularVel = (float)twistMessage.angular.z * angularSpeed;
            rb.angularVelocity = new Vector3(0, angularVel, 0);
        }
    }
    ```
    *Note*: You might need to adjust `linear.y` to `linear.z` depending on your robot's forward direction in Unity.

5.  **Set up the ROS TCP Connector in Unity**.
    -   Create an empty GameObject named `ROSConnection`.
    -   Add `ROSConnection` component to it (`Add Component > ROS > ROSConnection`).
    -   Configure the `ROS IP` (your ROS 2 machine IP) and `Unity IP` (your Unity machine IP if separate, or localhost).

6.  **Create a ROS 2 `cmd_vel` publisher node** (Python) in your ROS 2 workspace.
    
    ```python
    # ~/ros2_ws/src/unity_control/unity_control/teleop_publisher.py
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import sys

    class TeleopPublisher(Node):
        def __init__(self):
            super().__init__('teleop_publisher')
            self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.get_logger().info('Teleop publisher started. Use WASD to control.')

        def timer_callback(self):
            twist = Twist()
            twist.linear.x = self.linear_x
            twist.angular.z = self.angular_z
            self.publisher_.publish(twist)

        # Add keyboard input handling (this is simplified, full implementation needs a separate thread or non-blocking input)
        # For this lab, you might manually set self.linear_x and self.angular_z for testing or use a proper teleop node
        # Example: self.linear_x = 0.5, self.angular_z = 0.2

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()

    # Simple manual control for demonstration (replace with proper keyboard listener for real use)
    print("To move: set node.linear_x = 0.5; node.angular_z = 0.0. To stop: node.linear_x = 0.0")
    print("Run rclpy.spin(node) in another thread or call node.timer_callback() manually")
    
    # For a real teleop, use something like: 
    # import threading
    # spinner = threading.Thread(target=rclpy.spin, args=(node,))
    # spinner.start()
    # # ... then in main thread handle keyboard input to set node.linear_x/angular_z
    # spinner.join()
    
    rclpy.spin(node) # This will block, for simple testing you might only want this

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ```
    *Don't forget to create a `setup.py` entry point for this node and build it.*

7.  **Run ROS 2 nodes and Unity**:
    -   Start your ROS 2 `teleop_publisher` node (after building and sourcing).
    -   Run your Unity scene.

### Expected Output

When the ROS 2 node publishes `Twist` messages on `/cmd_vel`, the Cube in Unity should move according to the linear and angular velocities. This demonstrates successful two-way communication (Unity subscribes to ROS 2 topics).

## Checkpoint Quiz

<details>
<summary>Question 1: What are the main advantages of using Unity for robotics simulation compared to Gazebo?</summary>

Unity offers high-fidelity rendering, a rich asset store, and strong extensibility (C# scripting) which makes it ideal for visually rich simulations, human-robot interaction, and rapid prototyping of complex environments, whereas Gazebo excels in physics accuracy and deep integration with ROS for traditional robotics.

</details>

<details>
<summary>Question 2: How does the ROS-Unity Bridge facilitate communication between Unity and ROS 2?</summary>

The ROS-Unity Bridge (or ROS TCP Connector) establishes a TCP connection, allowing Unity to send and receive standard ROS 2 messages (topics, services, actions) over the network, effectively making Unity a ROS 2 node.

</details>

## References & Further Reading

- Unity Robotics Hub: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- ROS-Unity Bridge Tutorials: [https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)

## Diagrams

```mermaid
graph TD
    ROS2Node[ROS 2 Teleop Publisher] -- "/cmd_vel (TwistMsg)" --> ROSConnection[Unity ROSConnection]
    ROSConnection --> RobotController[Unity RobotController Script]
    RobotController --> Cube[Unity Cube (Rigidbody)]
```
