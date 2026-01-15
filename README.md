# EKF-SLAM in ROS/Gazebo
This repository implements an Extended Kalman Filter (EKF) SLAM pipeline for a TurtleBot in a ROS–Gazebo simulation. 
The robot can localize itself and build a map of an unknown environment by detecting cylindrical pillars as point landmarks.

## Problem Formulation
The robot operates in an unknown 2D environment and must simultaneously estimate:
- Its own pose (x, y, θ)
- The positions of static landmarks
using odometry and landmark observations.

## System Overview
- Robot: Turtlebot3 (Differential Drive)
- Simulator: Gazebo
- Sensors: Simulated range/bearing landmark observations
- Environment: Gazebo environment with cylindrical pillars
- Visualization: RViz for estimated pose, landmarks, and uncertainty ellipses

<img width="2292" height="1291" alt="image" src="https://github.com/user-attachments/assets/f9c349f8-0881-4e7d-bac1-47fe5fcdc08f" />

## EKF-SLAM Method
- State vector: Robot pose + Landmark positions
- Motion model: Wheel odometry
- Measurement model: Range and bearing to landmarks
- Data association: Handles unknown correspondence of measurements
- Filter:EKF is used for joint estimation of robot pose and landmark positions with uncertainties
This allows to incrementally improve its pose and landmarks' estimates as more observations are collected


## Dependencies
This project relies on the official TurtleBot3 ROS packages for simulation and manual control, which must be installed separately:

## Running the simulation
Launch the main.launch file with roslaunch ekf_slam_sim main.launch
This will automatically start:
- Gazebo simulation
- Remote control of the robot (via w, a, s, d)
- RViz visualization
- EKF-SLAM algorithm

## Results
- Accurate estimation of robot pose and landmark positions
- Uncertainty reduces over time as observations accumulate

<img width="2292" height="1297" alt="image" src="https://github.com/user-attachments/assets/f6ec8cbf-4d42-4abf-9ff3-5f601e02b12e" />

<img width="2288" height="1291" alt="image" src="https://github.com/user-attachments/assets/5a6ec549-1432-4779-b616-0148fe5dd1f2" />





