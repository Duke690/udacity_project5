# Home Service Robot
This document is a short writeup of the packages used for Mapping, Localization and Navigation in this project.

## Mapping
RTAB-Map Package
http://wiki.ros.org/rtabmap_ros

RTAB-Map is a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. It is used to create a 2D occupancy grid map that is then used for navigating the robot.

## Localization
AMCL Package
http://wiki.ros.org/amcl

A probabilistic localization system for a robot moving in 2D. It implements the adaptive Monte Carlo localization approach, which uses a particle filter to track the pose of a robot against a known map.

## Navigation
ROS Navigation Stack
http://wiki.ros.org/turtlebot_navigation

The ros navigation stack is used for path planning and execution with obstacle avoidance.