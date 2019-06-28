# XavierAutonomouDriving
ROS-based program to control a 1/3-scale motorized vehicle.

## Hardware
* [on-board computer] NVIDIA Xavier, Ubuntu 18, JetPack 4.2
* [2D LiDAR] RPLIDAR A1 https://www.slamtec.com/en/Lidar/A1
* [motor driver] 5V-36V 350W DC Brushless Motor Controller BLDC PWM Driver Board
* [motor] 8inch 24V Brushless Hub Motor Toothless Wheel For Electric Scooter Skateboard
* [camera1] Logitech C270
* [camera2] ELP USB with Camera 2.1mm Lens 1080p Hd Free Driver USB Camera Module,2.0 Megapixel(1080p) Usb Camera,for Linux Windows Android Mac Os
* [PWM board] MCP4728 Evaluation Board
* [IMU] PCA9685
* [Joystick] SONY DUALSHOCK 3 + Mayflash Magic-NS (bluetooth receiver + converter)

## Features
* Joystick -> Magic-NS -> PWM board -> motor driver boards -> motors
* 2 front wheels + 1 auxiliary rear wheel. Only front wheels will be motorized
* SLAM using Google Cartographer 

## Getting started
These instructions will get you a copy of the project up and running on your local Ubuntu 18 machine for development and testing purposes.


### Prerequisites
1 Installing ROS melodic on your Ubuntu 18 machine
```
http://wiki.ros.org/melodic/Installation/Ubuntu
```
2 Install RPLIDAR_ROS
```
sudo apt-get install ros-melodic-rplidar-ros

or

https://github.com/robopeak/rplidar_ros
```
3 Install usb-cam ROS package
```
sudo apt-get install ros-melodic-usb-cam
```
usb-cam package may require some workarounds to function properly in ROS melodic.
```
https://github.com/ros-perception/image_pipeline/issues/201
```
4 Install SMBUS2 for Python2
ROS currently works with Python 2 only. To let non-root users to access i2c port of Xavier, you may need to add your user to i2c group. If necessary, copy troubleshoot/30-i2c-tools to /etc/udev/rules.d and reboot.

5 Install IMU packages
A tool for debugging IMU
[RTIMULib2](https://github.com/RTIMULib/RTIMULib2)
A ROS interface for the debugging tool mentioned above
https://github.com/jeskesen/i2c_imu

6 Follow the instruction below to install Google Cartographer. Because cartographer uses ninja as a compilation tool, it is recommended to create an independent ROS workspace for Google Cartographer and source the directory in .bashrc.
```
https://google-cartographer-ros.readthedocs.io/en/latest/
```


### Wiring/connection
1 Xavier GPIO <-> MCP4728
GPIO 2 <-> VDD
GPIO 6 <-> GND
GPIO 3 <-> SDTA
GPIO 5 <-> SCLK

2 MCP4728 <-> motor drivers
OutputA <-> left wheel speed
OutputB <-> left wheel direction
OutputC <-> right wheel speed
OutputD <-> right wheel direction

3 Magic-NS
Plug Magic-NS in the USB hub. Run in Direct Input Mode (green LED)

4 Cameras
Plug two cameras in the USB hub.

5 Lidar
Plug in typeC port.


### Running the test
```
roslaunch main main.launch
```

### IMU in Google Cartographer
For 2D LIDAR and SLAM, IMU is not a must. However, it can improve the quality of the map greatly especially at the initial stage of mapping. To enable IMU, cartographer requires an IMU topic (sensor_msgs/IMU).
ALso enable imu in "trajectory_builder.lua" in cartographer workspace.
