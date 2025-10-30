# Capstone_Cleaning_robot
Watch my robot here: https://www.youtube.com/watch?v=gQMT6rC0Xig

## Tables of content
- [Technologies](#technologies)
- [Features](#features)
- [Usages](#Usages)
- [Contributions](#contributions)

### Technologies
In this thesis, we design a differential robot with a robotic arm mounted on top of the base. We focus on developing the design and control system for the robot arm with the ability to pick up and drop multiple types of objects (including milk cartons, crumpled paper, small carton boxes, cups, beer cans, and water bottles).
To achieve this, we researched and implemented various robotics and AI techniques such as:
* Forward / Inverse Kinematics
* Robot Dynamics modeling
* Extended Kalman Filter (EKF) for state estimation
* PI Velocity Controller
* ROS 2 (Robot Operating System) for system integration
* YOLOv11-nano for real-time object detection

In addition, a navigation algorithm was developed so that the differential robot can autonomously follow a zigzag trajectory in a mapped environment. All modules — manipulation, navigation, and perception — are integrated to allow the robot to detect objects and perform cleaning tasks efficiently.

### Features
* Autonomous navigation using zigzag path planning
* Real-time object detection with YOLOv11-nano
* Capable of picking up and placing different types of waste (cartons, bottles, cans, paper, etc.)
* Robotic arm with forward & inverse kinematics control
* PI-based velocity controller for wheel motion
* State estimation using Extended Kalman Filter (EKF)
* Fully integrated in ROS 2 (Humble)

