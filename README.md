# Comprehensive Guide: Running SLAM with RPLidar A2/A3 on ROS2

This guide walks you through the installation and execution of SLAM using the RPLidar A2/A3 on ROS2, leveraging the `rf2o_laser_odometry` and `turtlebot4` packages for odometry and visualization.

![ROS Badge](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ROS&logoColor=white)
![Robot Framework Badge](https://img.shields.io/badge/SLAMTEC%20Lidar-000000?style=for-the-badge&logo=robot-framework&logoColor=white)
![ROS2 Humble](https://img.shields.io/badge/ROS2:Humble-0078D7?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

---

## Step 1: Install ROS2

If you haven't installed ROS2 yet, follow the official instructions for your distribution. This guide assumes you're using ROS2 Humble or any other supported version.

1. Follow the official ROS2 installation instructions from [here](https://docs.ros.org/en/humble/Installation.html).
   
2. Ensure your workspace is set up:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2: Install RPLidar Dependencies

### 2.1 Install RPLidar ROS2 Package

You'll need to install the `sllidar_ros2` package, which contains the drivers to interface with the RPLidar A2/A3.

1. Clone the `sllidar_ros2` repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

2. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

3. Ensure the RPLidar is connected to your machine via USB. You may also need to set the correct permissions for the USB port. Run:

```bash
sudo chmod 666 /dev/ttyUSB0
```

### 2.2 Test the RPLidar Node

To ensure the RPLidar node is functioning correctly:

1. Start the RPLidar node:

```bash
ros2 launch sllidar_ros2 view_sllidar_a3_launch.py
```

2. Confirm that scan data is being published:

```bash
ros2 topic echo /scan
```

---

## Step 3: Install Odometry Package (`rf2o_laser_odometry`)

Odometry is essential for accurate SLAM. Install the `rf2o_laser_odometry` package to calculate odometry using the LIDAR scan data.

1. Install and build the package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
```
Now build the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select rf2o_laser_odometry
```

2. Start the odometry node:

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py laser_scan_topic:=scan
```

This will process the LIDAR data from the `/scan` topic and compute odometry.

---

## Step 4: Set Up Static Transformations

You need to publish static transformations between `base_link`, `laser`, and `odom` frames to ensure proper frame alignment for SLAM.

1. Publish the transform between the `base_link` and `laser`:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```

2. Publish the transform between the `base_link` and `odom`:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link odom
```

These transforms are necessary to ensure the SLAM algorithm can accurately interpret the robot's position.

---

## Step 5: Install the TurtleBot4 Package for SLAM

The TurtleBot4 package provides a SLAM implementation that can be used with any robot setup. It includes SLAM algorithms and visualization tools.

### 5.1 Install the TurtleBot4 Package

Follow the installation instructions from the official [Turtlebot4 Documentation](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html).

```bash
sudo apt install ros-<ros2-distro>-turtlebot4*
```

### 5.2 Launch the SLAM Algorithm

First, you need to find the path to the `slam.yaml` file in your `turtlebot4_navigation` package. Run the following command to find the installation path of the `turtlebot4_navigation` package

```bash
ros2 pkg prefix turtlebot4_navigation
```
Navigate to the `config` directory within this package to locate the `slam.yaml` file. 

Once you have the path to `slam.yaml`, you can lauch the the slam algorithm using the following command, replacing the `/path/to/slam.yaml` with the actual path to your SLAM configuration file:

```bash
ros2 launch turtlebot4_navigation slam.launch.py params:=/full/path/to/slam.yaml
```
This starts the SLAM process and allows you to map the environment using LIDAR data.

---

## Step 6: Visualize the Robot in RViz

To visualize the robot and SLAM output in RViz:

1. Launch the robot visualization:

```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

2. This will open an RViz window where you can see the LIDAR scans, robot position, and SLAM map being generated.

---

## Step 7: Troubleshooting

### 7.1 Check for Scans

Make sure that the LIDAR is publishing scan data:

```bash
ros2 topic echo /scan
```

If the scan data is not being published, double-check the RPLidar connection and permissions.

### 7.2 Verify Odometry

If you encounter issues with odometry, verify that the `rf2o_laser_odometry` node is running correctly and that the topic `/odom` is being published.

```bash
ros2 topic echo /odom
```

### 7.3 Transform Issues

If you get errors about missing transforms, ensure you have correctly published the static transforms for `base_link`, `laser`, and `odom`.

---

## Conclusion

By following these steps, you should have a complete SLAM system running with the RPLidar A2/A3, utilizing odometry and the Turtlebot4 package for navigation and visualization. This setup will allow you to map your environment in real-time and monitor the SLAM process visually.

If you encounter any issues, refer to the troubleshooting steps or ROS2 logs for debugging, or raise the **Pull Request**.

---
## Contributors

![GitHub Contributors](https://img.shields.io/github/contributors/SYED-M-HUSSAIN/Ros2-Slam-RPlidar?style=for-the-badge)
- [Syed Muhammad Hussain](https://github.com/SYED-M-HUSSAIN)  
- [Sadiqah Mushtaq](https://github.com/SadiqahMushtaq)  
