<div align="center">

# 🎯 RF2O Laser Odometry

*ROS 2 Jazzy · C++ · Fast 2D Planar Odometry*

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![C++](https://img.shields.io/badge/C++-14-yellow?style=flat-square&logo=c%2B%2B)
![License](https://img.shields.io/badge/License-GPL_v3-green?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-purple?style=flat-square&logo=ubuntu)

> **Original Authors** — MAPIR UMA  
> **Paper** — [Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach](http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf) (ICRA 2016)

</div>

---

## Overview

Estimation of 2D odometry based on planar laser scans. rf2o is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans. It is particularly useful for mobile robots with inaccurate wheel odometry.

For every scanned point, we formulate the range flow constraint equation in terms of the sensor velocity, and minimize a robust function of the resulting geometric constraints to obtain the motion estimate. Conversely to traditional approaches, this method does not search for correspondences but performs dense scan alignment based on the scan gradients, in the fashion of dense 3D visual odometry.

The very low computational cost (0.9 milliseconds on a single CPU core) together with its high precision, makes RF2O a suitable method for robotic applications that require planar odometry.

---

## Integration in Multi-TurtleBot3 Convoy System

This package has been fully integrated into the `multi_tb3_system` workspace to provide high-precision laser-based odometry for the leader and follower robots. 

By setting `enable_rf2o:=true` (default) during launch, each TurtleBot3 spawns its own namespaced `rf2o_laser_odometry_node`. This substantially improves path-following accuracy by decoupling tracking from pure wheel-slip-prone kinematic models in Gazebo.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select rf2o_laser_odometry
source install/setup.bash
```

---

## Quick Start (Standalone)

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

---

## ROS Topic Architecture

| Topic | Type | Direction | Description |
|---|---|:---:|---|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe | Input from 2D LiDAR |
| `/odom_rf2o` | `nav_msgs/Odometry` | Publish | Estimated odometry |

*Note: In the `multi_tb3_system`, these topics are remapped dynamically by `spawn_robots.launch.py` to match each robot's namespace (e.g., `/tbX/scan` and `/tbX/odom`).*

---

## Project Structure

```
rf2o_laser_odometry/
├── include/rf2o_laser_odometry/    # C++ Headers
│   ├── CLaserOdometry2D.hpp        # Core RF2O algorithm
│   └── CLaserOdometry2DNode.hpp    # ROS 2 Node wrapper
├── src/                            # C++ Source
│   ├── CLaserOdometry2D.cpp
│   └── CLaserOdometry2DNode.cpp
├── launch/
│   └── rf2o_laser_odometry.launch.py
├── CMakeLists.txt
└── package.xml
```
