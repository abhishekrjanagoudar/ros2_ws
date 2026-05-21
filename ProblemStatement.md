<style>
@keyframes colorPulse {
  0%, 100% { color: #2c3e50; }
  50% { color: #34495e; }
}
.animated-title {
  animation: colorPulse 4s infinite;
}
</style>

<div align="center">
  <h1 class="animated-title">Multi-Robot TurtleBot3 Burger Leader–Follower System with LiDAR-Based Target and Obstacle Differentiation in Simulation</h1>
  <h3>In a Simulated Environment</h3>
  <br>
  <h4>Assignment for University: Hochschule Darmstadt</h4>
  <p>
    <strong>Under the guidance of:</strong><br>
    <b>Prof. Dr.-Ing. Karl Kleinmann</b><br>
    <i>Fachgebiet Automatisierungssysteme</i><br>
    <i>Fachbereich Elektrotechnik und Informationstechnik</i><br>
    <b>Hochschule Darmstadt - University of Applied Sciences</b>
  </p>
</div>

<hr>

## 🎯 Problem Overview

The goal of this project is to design and implement a ROS-based multi-robot system using **TurtleBot3 Burger** robots in a **Gazebo simulation environment**. The system consists of multiple robots operating in a coordinated convoy formation within a shared environment.

The first robot is manually controlled using teleoperation, while the subsequent robots autonomously follow the preceding robot using **LiDAR (LaserScan)** data. The key challenge is enabling the follower robots to reliably distinguish between the leading TurtleBot3 Burger (dynamic target) and static environmental objects such as columns or pillars, using only onboard sensor data.

---

## ⚙️ Core System Behavior

- The first TurtleBot3 Burger is teleoperated using standard ROS teleoperation tools.
- The second and third TurtleBot3 Burgers act as autonomous followers.
- Each follower robot must:
  - Track and follow the robot immediately ahead
  - Maintain a safe and stable distance and velocity
  - Continuously process LiDAR data for decision-making

---

## ⚠️ Critical Technical Challenge (Key Requirement)

The system must differentiate between:

**1. Moving Target (Lead TurtleBot3)**
- Dynamic object
- Changes position over time
- Must be identified and tracked as the primary reference for following behavior

**2. Static Objects (Environment)**
- Columns, pillars, and other obstacles in the Gazebo world
- Stationary or non-moving structures
- Must be detected and treated as obstacles to avoid, not as follow targets

### 🧠 Required Distinction Logic

The follower robots must use LaserScan data to:
- Identify dynamic motion patterns (leader robot)
- Separate them from static LiDAR returns (columns/walls)
- Ensure that only the robot ahead is used as the tracking target, while static objects are handled as obstacles

This requires combining:
- Distance continuity over time
- Angular position consistency
- Motion-based filtering or tracking logic

---

## 🚀 Objectives

- **Develop** a ROS system with multiple nodes and packages for multi-robot coordination.
- **Simulate** TurtleBot3 Burger robots in Gazebo.
- **Implement** leader–follower behavior:
  - **Robot 1 →** teleoperated control
  - **Robot 2 & 3 →** autonomous following using LiDAR-based perception
- **Implement** target vs obstacle classification using sensor data.
- **Maintain** safe inter-robot spacing and velocity control.
- **Ensure** collision-free navigation in environments with static obstacles.

---

## 🌍 Environment Setup

- Empty Gazebo world for baseline formation testing.
- Extended environment with static columns/pillars for obstacle scenarios.
- Robots must operate reliably in both environments.

---

## 🚧 Key Challenges

- Correctly distinguishing the leader robot from static objects using only LiDAR.
- Avoiding false tracking of columns or walls as the leader.
- Maintaining stable convoy behavior without oscillations.
- Handling sensor noise and occlusions in multi-robot scenarios.
- Ensuring real-time performance in Gazebo simulation.

---

## 🎉 Expected Output

A working ROS multi-robot simulation where:
- Robot 1 is teleoperated.
- Robot 2 and 3 follow Robot 1 sequentially.
- Followers reliably track the leader robot only.
- Static objects (columns) are detected and avoided, not mistaken as targets.
- Stable formation movement with collision avoidance.

---

## 📦 Deliverables

- **ROS workspace** with code (nodes + packages)
- **Gazebo simulation worlds** (empty + obstacle environment)
- **System documentation**
- **Test cases** and evaluation results
- **Presentation** (~50 slides) including:
  - Problem statement
  - System architecture
  - Target vs obstacle discrimination logic
  - Simulation results and demos