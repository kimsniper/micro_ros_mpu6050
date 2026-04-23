# micro_ros_mpu6050

A Micro-ROS based IMU driver for ESP-IDF using MPU6050 with an Extended
Kalman Filter (EKF) for orientation estimation.

------------------------------------------------------------------------

## Overview

This project integrates:

-   **MPU6050 IMU (accelerometer + gyroscope)**
-   **EKF (Extended Kalman Filter) for orientation estimation**
-   **micro-ROS on ESP-IDF**
-   **ROS 2 compatible IMU publisher (`sensor_msgs/Imu`)**
-   **FreeRTOS task-based real-time streaming**

The ESP32 reads IMU data, fuses it using EKF, and publishes orientation
data via micro-ROS over UDP to a ROS 2 agent.

------------------------------------------------------------------------

## Features

-   Real-time IMU data streaming
-   Quaternion + Euler angle estimation
-   Gyro bias calibration at startup
-   Accelerometer validation filtering
-   EKF-based attitude estimation
-   micro-ROS UDP transport
-   ROS 2 `/imu/data` topic publishing

------------------------------------------------------------------------

## Hardware Requirements

-   ESP32 / ESP32-S3 / ESP32-C3 (ESP-IDF supported)
-   MPU6050 IMU module
-   I2C connection

### Wiring (default config)

  MPU6050   ESP32 GPIO (default)
  --------- ----------------------
  SDA       GPIO 18 / 5 / 18
  SCL       GPIO 19 / 6 / 19
  VCC       3.3V
  GND       GND

------------------------------------------------------------------------

## Micro-ROS Architecture

-   ESP32 runs micro-ROS client
-   UDP transport via WiFi
-   ROS 2 agent bridges communication
-   Publishes: `/imu/data` (`sensor_msgs/msg/Imu`)

------------------------------------------------------------------------

## Setup Instructions

### 1. Clone Project

``` bash
cd ~
git clone --recursive https://github.com/kimsniper/micro_ros_mpu6050.git
cd ~/micro_ros_mpu6050
```

------------------------------------------------------------------------

### 2. Install ESP-IDF

Follow official ESP-IDF setup guide:
https://docs.espressif.com/projects/esp-idf/en/latest/

------------------------------------------------------------------------

### 3. Set Target

``` bash
idf.py set-target esp32
# or esp32s3 / esp32c3 depending on board
```

------------------------------------------------------------------------

### 4. Configure Project

``` bash
idf.py menuconfig
```

------------------------------------------------------------------------

## Micro-ROS Configuration (menuconfig)

### Micro-ROS MPU6050 Setting

    Menu: Micro-ROS MPU6050 Setting

-   I2C SCL GPIO → default depends on chip
-   I2C SDA GPIO → default depends on chip
-   I2C Frequency → 400000
-   micro-ROS Task Stack → 16000 bytes
-   micro-ROS Task Priority → 5

------------------------------------------------------------------------

## micro-ROS Network Settings

Inside menuconfig:

    Micro-ROS Settings → WiFi Configuration

Set:

-   WiFi SSID
-   WiFi Password
-   micro-ROS Agent IP (PC running ROS2)
-   micro-ROS Agent Port → 8888

Example:

    Agent IP: 192.168.1.10
    Port: 8888

------------------------------------------------------------------------

## Build & Flash

``` bash
idf.py build
idf.py flash
idf.py monitor
```

------------------------------------------------------------------------

## Run micro-ROS Agent

Before running ESP32, start the agent:

``` bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

------------------------------------------------------------------------

## ROS 2 Usage

After successful connection:

### Check topic

``` bash
ros2 topic list
```

### View IMU data

``` bash
ros2 topic echo /imu/data
```

------------------------------------------------------------------------

## Output Data

Published message:

-   Orientation (Quaternion)
-   Angular velocity (rad/s)
-   Linear acceleration (m/s²)

------------------------------------------------------------------------

## Notes

-   Keep IMU still during startup calibration
-   Ensure stable WiFi connection
-   EKF improves stability over raw MPU6050 data
-   Gravity-based validation filters noisy accelerometer readings

------------------------------------------------------------------------

## Troubleshooting

### No data received

-   Check agent is running
-   Verify IP address in menuconfig
-   Check WiFi connection

### Wrong orientation

-   Recalibrate gyro (restart device)
-   Ensure IMU is fixed during calibration

------------------------------------------------------------------------

## Integrate ROS2 IMU Demo

``` bash
cd ~
git clone https://github.com/kimsniper/ros2_imu_demo.git
cd ~/ros2_imu_demo
```

### Build project

``` bash
colcon build
```

### Source

``` bash
source install/setup.bash 
```

### Launch

``` bash
ros2 launch ros2_imu_demo imu_demo.launch.py
```

------------------------------------------------------------------------

## License

BSD 3-Clause License

Copyright (c) 2026 Mezael Docoy
