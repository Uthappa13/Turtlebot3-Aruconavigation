# Intelligent Warehouse Robotics: Aruco-Assisted Navigation

## **Project Overview**
This project focuses on developing a **ROS2 package** that enables a **TurtleBot** to navigate a warehouse environment using **Aruco markers** for guidance. 

While navigating, the robot detects and reports objects present in the environment. The key objectives include:
- Moving the robot through the maze by detecting **Aruco markers**.  
- Executing predefined actions based on marker identification.  
- Detecting and reporting the **pose of objects** in the environment.  

## **Motivation**
Autonomous robot navigation is essential for **industry applications**, such as warehouse automation and search-and-rescue operations. This project demonstrates:
 **Real-time perception** using Aruco markers  
 **Autonomous decision-making** in navigation  
 **Efficient object detection** for real-world deployment  
 **ROS2-based modular implementation**  

## **Tech Stack**
- **ROS2 (Robot Operating System)**  
- **C++ (rclcpp)** for writing ROS2 nodes  
- **Gazebo** for robot simulation  
- **OpenCV** for **Aruco marker detection**  
- **TF2** for coordinate transformations  
- **std_msgs, geometry_msgs, mage_msgs** for ROS2 message passing  

## **Project Workflow**
This environment includes:
 - A **TurtleBot** equipped with an **RGB camera** and a **logical camera**.
 - **Aruco markers** placed on the walls for navigation guidance.
 - **Objects** placed in the environment for detection.

**Robot Navigation Logic**
The robot follows this workflow:
 - Move forward until an Aruco marker is detected.
 - Stop at ≤ 0.4m from the detected marker.
 - Perform an action based on the marker ID:
 - Continue navigating until the end marker is detected.
 - Upon reaching the end, detect and report the pose of batteries.

 **Object Detection and Pose Estimation**
 - The **RGB camera** detects **Aruco markers** and publishes data to */aruco_markers*.
 - The **logical camera** detects floating **objects** and publishes to */mage/advanced_logical_camera/image*.


