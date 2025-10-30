# Capstone_Cleaning_robot
***Watch my robot here***: https://www.youtube.com/watch?v=gQMT6rC0Xig

## Tables of content
- [Technologies](#technologies)
- [Features](#features)
- [Demo](#demo)
- [Developers](#developers)

### Technologies
In this thesis, we design a differential robot with a robotic arm mounted on top of the base. We focus on developing the design and control system for the robot arm with the ability to pick up and drop multiple types of objects (including milk cartons, crumpled paper, small carton boxes, cups, beer cans, and water bottles).
To achieve this, we researched and implemented various robotics and AI techniques such as:
* Forward / Inverse Kinematics of Differential mobile robot, 4 DOF robot arm
* Robot Dynamics modeling to calculate moments / torque on the robot arm
* Extended Kalman Filter (EKF) for heading estimation (fusing kinematic odometry and IMU yaw)
* PI Velocity Controller using DS (Direct Synthesis) method
* ROS 2 (Robot Operating System) for system integration
* YOLOv11-nano for real-time object detection
* **Devices**:
  * 2 × JGB37-520 DC servo motors for mobile platform
  * Hiwonder bus servos: 1 × HX-06L, 1 × LX-225, 2 × LX-15D, 1 × MG90S for robot arm and gripper
  * Raspberry pi 5: running ROS2, EKF, processing image, and communicating with STM32 microcontrollers
  * 1 STM32F103C8T6 bluepill: control DC servo motors and send data via UART for Raspberry to calculate odometry. [Code here](https://github.com/Phat-sv/DC_servo_motor_STM32)
  * 1 STM32F103C8T6 bluepill: read data from VL53L1X sensor on mobile platform (identify distance to object), IMU BNO055 and send to Raspberry via UART [Code here](https://github.com/Phat-sv/VL53L1X_and_BNO055_STM32)
  * 1 STM32L432KCU6 (nucleo): read data from VL53L1X sensor mounted on gripper and calculate object coordinates. Moving to object and pick up, after that, sending result to Raspberry [Code here](https://github.com/Phat-sv/4_DOF_Robot_Arm_STM32)
  * AI Hailo-8L: running yolo-v11
  * Lidar A1M8: mapping and navigation
  * IMU BNO055: yaw measurement
  * Raspberry Pi Camera Module V3: object detection

### Features
* Autonomous navigation using zigzag path planning
* Real-time object detection with YOLOv11-nano
* Capable of picking up and placing different types of waste (cartons, bottles, cans, paper, etc.)
* Robotic arm with forward & inverse kinematics control
* PI-based velocity controller for wheel motion
* State estimation using Extended Kalman Filter (EKF)
* Fully integrated in ROS 2 (Humble)

### Demo
* Yolov11-nano: [video](https://youtu.be/JAwOVxa2ZUE)
* Object detection and gripping: [video](https://youtube.com/shorts/5fRmE0WF7jY)
* Zig zag trajectory: [video](https://youtu.be/SWXLe86SoOQ), [picture](https://github.com/Phat-sv/Capstone_Cleaning_robot/blob/main/images/Zig-zag%20result.png)
* Matlab simulation for robot arm: [video](https://youtu.be/U4f-FVTDqog)
* Gripping and navigation: [video](https://youtu.be/RqK6kfQlSYY)
* EKF odom vs raw odom: [picture](https://github.com/Phat-sv/Capstone_Cleaning_robot/blob/main/images/EKF%20odom.png)
* Velocity response (PI algorithm): [picture](https://github.com/Phat-sv/Capstone_Cleaning_robot/blob/main/images/PID%20response.png)
  
### Developers
1. [Nguyen Tan Phat](https://github.com/Phat-sv) — responsible for STM32 programming, deploying algorithms (PID, EKF), kinematics, training YoloV11-nano
2. [Phung Hieu Cuong](https://github.com/phcfg12) — responsible for PCB design, 3D modeling (Inventor), ROS 2 configuration, and deploying YOLOv11-nano on Hailo-8L


