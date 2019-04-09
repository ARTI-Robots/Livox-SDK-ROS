# Livox ROS Demo

Livox ROS demo is an application software running under ROS environment. It supports point cloud display using rviz. The Livox ROS demo includes two software packages, which are applicable when the host is connected to LiDAR sensors directly or when a Livox Hub is in use, respectively. This Livox ROS demo supports Ubuntu 14.04 (ROS Indigo)/Ubuntu 16.04 (ROS Kinetic), both x86 and ARM. It has been tested on Intel i7 and Nvidia TX2. 

**NOTE**: Every Livox LiDAR unit has its unique broadcast code.The broadcast code consists of its serial number and an additional number (1,2, or 3). The serial number can be found on the body of the LiDAR unit (below the QR code). The detailed format is shown as below:

![broadcast_code](./broadcast_code.png)

## Livox ROS Demo User Guide

The Livox-SDK-ROS directory is organized in the form of ROS workspace, and is fully compatible with ROS workspace. A subfolder named ***src*** can be found under the Livox-SDK-ROS directory. Inside the ***src*** directory, there are two ROS software packages: display_lidar_points and display_hub_points.

### Compile & Install Livox SDK 

1. Download or clone the [Livox-SDK/Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/) repository on GitHub. 

2. Compile and install the Livox SDK under the ***build*** directory following `README.md` of Livox-SDK/Livox-SDK

### Run ROS Demo and Display PointCloud by rviz 

1. Download or clone the code from the Livox-SDK/Livox-SDK-ROS repository on GitHub. 

    

2. Compile the ROS code package under the Livox-SDK-ROS directory by typing the following command in terminal:
    ```
     catkin_make
    ```

3. Source setup.bash file:
    ```
     source ./devel/setup.bash
    ```

4. Enter broadcast code from command line.
    ```
     roslaunch display_lidar_points livox_lidar.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
    ```
     or
     ```
     rosrun display_hub_points livox_hub.launch bd_list:="hub_broadcast_code"
     ```

### Run livox ros driver

livox_ros_driver is a new ros package under the Livox-SDK/Livox-SDK-ROS/src directory, which is designed to gradually become the standard driver package for livox devices in the ros environment. The driver offers users a wealth of options when using different launch file. There is *bd_list* arg in each launch file. All Livox LiDAR units in your LAN will be connected automatically in default.

e.g.

```
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

If you want to connect to the specific LiDAR uint(s) only, please add the broadcast code into command line. 

e.g.

```
roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
```

Features of *.launch* files are listed as below:

| launch file               | features                                                     |
| ------------------------- | ------------------------------------------------------------ |
| *livox_lidar_rviz.launch* | Connect to Livox LiDAR units<br/>Publish pointcloud2 format point cloud<br/>Automatically load rviz |
| livox_hub_rviz.launch     | Connect to Livox Hub units<br/>Publish pointcloud2 format point cloud<br />Automatically load rviz |
| livox_lidar.launch        | Connect to Livox LiDAR units<br />Publish pointcloud2 format point cloud |
| livox_hub.launch          | Connect to Livox Hub units<br />Publish pointcloud2 format point cloud |
| livox_lidar_msg.launch    | Connect to Livox LiDAR units<br />Publish livox custom format point cloud |
| livox_hub_msg.launch      | Connect to Livox Hub units<br />Publish livox custom format point cloud |

Livox custom msg format：

```
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```
pointcloud format:
```
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar
```
