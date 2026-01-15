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
- Tools: RViz for visualization of the estimations and their uncertainties

<img width="2292" height="1291" alt="image" src="https://github.com/user-attachments/assets/f9c349f8-0881-4e7d-bac1-47fe5fcdc08f" />

## EKF-SLAM Method
- State vector: Robot pose + Landmark positions
- Motion model is determined by wheel odometry
- Measurement model is based on range and bearing to landmarks
- Alongside measurement model, a proccess of data association is done due to unknown correspondency of measurements
- EKF is used for the joint state and map estimation with their uncertainties


## Dependencies
This project relies on existing TurtleBot3 ROS packages for simulation and manual control, which are not included in this repository.

These packages must be installed separately following the official TurtleBot3 ROS installation instructions.

## How to run
Launch the main.launch file. This will automatically start the Gazebo simulation, the remote control of the robot, the RViz tool and the EKF-SLAM algorithm.
Robot is controlled with the w, a, s, d keys.

## Results
- Estimations on robot pose and landmark positions are done accurately
- Estimations improve over time with a reduction of uncertainty as observations accumulate

<img width="2292" height="1297" alt="image" src="https://github.com/user-attachments/assets/f6ec8cbf-4d42-4abf-9ff3-5f601e02b12e" />

<img width="2288" height="1291" alt="image" src="https://github.com/user-attachments/assets/5a6ec549-1432-4779-b616-0148fe5dd1f2" />





