# TurtleBot3 Multi-Robot Control & Navigation (ROS 2)

This repository contains ROS 2 packages and launch files for running TurtleBot3 in both simulation and real hardware, supporting multi-robot setups, teleoperation, circular motion control, and straight-line obstacle avoidance with lane switching.

## Setup

Run the following commands in **every terminal** before launching any node:

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
