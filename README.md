# SLAM System with IMU and Wifi Synchronization

## Introduction:
For a better functionality of the SLAM system, we collect the accelerometer, gyroscope, and GPS readings of the phone which would be helpful in better navigation, motion tracking, and orientation. For implementng this into our system, we use the SensorServer application. This Android app allows us to stream real-time sensor data from the phone to multiple WebSocket clients, enabling them to monitor the device's movement, environment, and position in real-time. 

Clients can connect to the app using a WebSocket URL to receive data from various sensors, such as the accelerometer and gyroscope, in JSON format. We use this app to obtain accelerometer and gyroscope readings from the phone and stream them to the server using WebSocket after connecting our phone via hotspot. This data is then processed by a ROS node, implemented in Python, which subscribes to the WebSocket server and publishes the sensor data as ROS topic.

## Table of Contents:
1. [Usage](#usage)
2. [Application Information](#application-information)
3. [Set Up](#set-up)
4. [Citations](#citations)

### Usage:
This project directory consists of two main directories:

1. [rf_msgs](./rf_msgs): A clone of [rf_msgs](https://github.com/ucsdwcsng/rf_msgs) message type from WiROS package.
    

2. [src](./src): The code to synchronize Wifi messages to the accelerometer, gyroscope, and GPS readings of the phone.
    - [gps_publisher.py](./src/gps_publisher.py): A ROS node that connects to a WebSocket server to receive GPS data and publish it as a ROS topic. It establishes a WebSocket connection to a specified URL (ws://localhost:8090/gps) and listens for incoming GPS data in JSON format, extracting latitude and longitude values. These values are then packaged into a NavSatFix message and published to the /gps ROS topic. The script also handles connection events, errors, and graceful shutdown on receiving a termination signal (Ctrl+C). Additionally, it periodically sends a "getLastKnownLocation" request to the WebSocket server to fetch the latest GPS coordinates.
    - [imu_publisher](./src/imu_publisher.py): A ROS node designed to collect and publish IMU (Inertial Measurement Unit) data from an Android device to a ROS topic. The script connects to a WebSocket server using a specified URL(ws://localhost:8090/sensors/connect?types={self.encoded_types}), retrieving accelerometer and gyroscope data. This data is buffered, synchronized based on timestamps, and then published as an Imu message to the /imu ROS topic. The node also handles errors, reconnection attempts, and shuts down gracefully when interrupted (Ctrl+C). The data synchronization ensures that the IMU data is aligned in time before being published.
    - [synchronizer.py](./src/synchronizer.py): The ROS launch file starts several nodes, including `imu_publisher` for IMU data, `gps_publisher` for GPS data, and basic operations from the `wiros_csi_node` package. It also launches a `synchronizer_node` to manage synchronization tasks between the different data sources.

### Application Information:
In order to collect the accelerometer, gyroscope and GPS readings, we use an application called [Sensor Server](https://github.com/umer0586/SensorServer). 

- There are various ways to retrieve the Inertial Measurement(IMU) and GPS readings and  from the phone. However, the imu_publisher package achieves it by connecting the Android phone to the device via hotspot.

### Set Up:
- To install this package, clone this repository in the catkin_ws directory and in the repository's root folder run catkin build:

        cd /home/user/catkin_ws/src
        git clone https://github.com/WS-UB/imu_publisher.git
        cd imu_publisher
        catkin build

- Source the packages and confirm the paths are included:

        source ./devel/setup.bash
        echo $ROS_PACKAGE_PATH

- Install the **SensorServer** application from [Installation](https://github.com/umer0586/SensorServer/blob/main/README.md#installation:~:text=connected%20android%20phone.-,Installation,-OR)
- Once the application is downloaded, open it and click on the menu bar in the top left corner. 
- Choose settings and enable the *Use Hotspot* Option. 
- Additionally, you can also change the Websocket Port Number, which should also be modified in the [imu_publisher](./src/imu_publisher.py) and [gps_publisher](./src/gps_publisher.py) files. The current port number in the files is 8090.
- Next, lauch the [synchronizer.launch](./launch/synchronizer.launch) file using the following command:
    
        roslaunch imu_publisher synchronizer.launch

- Go back to the *Server* in the bottom left corner and click on start. 
- If the connection is established, you will see a pop up notification on the application in the middle bottom at the *Connections* icon. Since we are retrieving the IMU data and GPS readings, we should be able to see two connections.

By following the above steps, we are able to synchronize the Wifi messages with the accelerometer, gyroscope and GPS readings.

### Citations:

SensorServer Application: https://github.com/umer0586/SensorServer/blob/main/README.md
