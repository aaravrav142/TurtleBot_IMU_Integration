# TurtleBot_IMU_Integration

TurtleBot IMU Integration
2016-03-18 00:00:00 +0000

Overview

The goal of this project was install an IMU on the TurtleBot and fuse the IMU sensor data with existing odometry data to gather a more accurate pose estimate. In doing so, I was able to develop knowledge, experience, and skills pertaining to the following topics:

ROS Nav Stack
Mapping
Localization
Local and Global Planning
3D Printing
IMU Calibration
Sensor Fusion Through Extended Kalman Filters (EKF)
Hardware

TurtleBot 2
Razor 9DOF IMU
TurtleBot 2

Razor 9DOF IMU













3D Printing and IMU Mounting

When mounting the IMU on the TurtleBot, an attempt was made to make the IMU as close as possible to coaxial with the TurtleBot base in order to minmize IMU translation due to pure rotation of the robot. To do this, a customer 3D mounting bracket was printed using the Ultimaker 2 3D printer.

Ultimaker 2Bracket








Bracket With IMU Fully Mounted IMU











Mounting Bracket .STL File 

Setup and Calibration

ROS Drivers, as well as full calibration procedures for the Razor 9DOF IMU can be downloaded from http://wiki.ros.org/razor_imu_9dof/

Sensor Integration

IMU and Odometry Integration was performed using the ROS package ‘Robot_Localization’. This package uses and Extended Kalman Filter to fuse multiple sensor streams into a pose estimate.

Fused Sensor Components:

Odometry
x position
y position
yaw
x velocity
y velocity
yaw velocity
IMU
yaw
x acceleration
Robot_Localization Launch File

Diagonal Covariance Matrix Values:

Odometry [x,y,z,roll,pitch,yaw]: 
[.1, .1, INF, INF, INF, .05]

IMU [roll,pitch,yaw], [vroll,vpitch,vyaw], [aroll,apitch,ayaw] 
[.0025, .0025, .0025], [.02, .02, .02], [.04, .04, .04]

Results

The robot was driven thorugh a series of motions, and using an overhead camera and the overhead_mobile_tracker package developed by Jarvis Shultz, a ground truth pose was measured and compared to pose output by /odom on it’s own, and the pose output by the fused sensor data on the /odometry/filtered topic.

Two trial runs are shown below. The first, a test of linear accuracy, shows the fused measurement noisy, but ultimately converging towards the measured pose. With further tuning of the covariance and process noise matrices, this can be improved upon. The second video shows a test of angular accuracy. Again we see the same noise issues as with the linear test. The key improvement here is when the robot comes to a stop at the end. Notice that at the time of stoppage, the fused pose is significantly closer to the measured pose than the odom on it’s own. As the robot remains stationary, the consistant odom measurements pull the fused pose back into line with what the odom is reporting; however, a common practice in localization is to pause the pose estimation when the robot is known to be stopped. If we do this, using the fused estimate at the time of stoppage should increase the accuracy of our pose estimation.

