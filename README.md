# Environment Setup

Run the following commands in **every terminal** before launching any
node:

    export TURTLEBOT3_MODEL=burger
    export ROS_DOMAIN_ID=30
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Hardware Connection (Real Robot)

Connect to the TurtleBot3 via SSH:

    ssh burger@<robot_ip_address>

Replace `<robot_ip_address>` with the actual IP address of the robot.

# Single RObot Simulation (Gazebo)

Launch a TurtleBot3 robots in Gazebo:

    export TURTLEBOT3_MODEL=burger 
    ros2 launch turtlebot3_gazebo empty_world.launch.py


# Multi-Robot Simulation (Gazebo)

Launch multiple TurtleBot3 robots in Gazebo:

    ros2 launch turtlebot3_gazebo multi_robot.launch.py

# Hardware Bringup (Namespaced Robot)

Bring up the real robot using namespace `TB3_1`:

    ros2 launch turtlebot3_bringup robot.launch.py namespace:=TB3_1

# Teleoperation (Keyboard Control)

Control the robot using keyboard teleoperation:

    ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1

# Circle Controller Node

Run the circle controller with configurable parameters:

    ros2 run tb3_circle_controller circle_controller \
      --ros-args \
        -p namespace:=TB3_1 \
        -p radius:=1.3 \
        -p linear_speed:=0.10 \
        -p direction:=ccw

**Parameters:**

-   `namespace` -- Robot namespace

-   `radius` -- Circle radius (meters)

-   `linear_speed` -- Linear velocity (m/s)

-   `direction` -- Clockwise (`cw`) or Counter-clockwise (`ccw`)

# Lane Switching / Obstacle Avoidance

Run the straight-line motion with obstacle avoidance:

    ros2 run tb3_straight_avoid straight_avoid

# Battery check

check the battery 

     ros2 topic echo /battery_state

# Obstical detect

detect the obstical and stop

     ros2 run tb3_obstacle_stop obstacle_stop_node --ros-args -p safe_distance:=0.7 -p detect_angle_deg:=90.0



# Notes

-   Ensure consistent namespaces across bringup, teleop, and
    controllers.

-   `ROS_DOMAIN_ID` must match across all machines.

-   Supports both simulation and real hardware.
