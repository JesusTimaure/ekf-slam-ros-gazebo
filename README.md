# EKF-SLAM in ROS/Gazebo
This repository implements an Extended Kalman Filter (EKF) SLAM pipeline for a TurtleBot in a ROS–Gazebo simulation. The robot is capable of localizing itself and build a map of an unknown environment by detecting cylindrical pillars as point landmarks by using a Lidar sensor.

## Problem Formualtion
The robot initially operates in an unkwnown 2D enviroment and must be capable of estimating simultaneously:
- Its own pose (x, y, θ)
- The positions of static landmarks
Through the use of odometry and landmark observation.

## System Overview
- Robot: Turtlebot3 (Differential Drive)
- Simulator: Gazebo
- Sensors: Simulated range/bearing landmark observations
- Environment: Gazebo environment with cylindrical pillars

Entire EKF-SLAM is implemented in ROS

<img width="2292" height="1291" alt="image" src="https://github.com/user-attachments/assets/f9c349f8-0881-4e7d-bac1-47fe5fcdc08f" />




