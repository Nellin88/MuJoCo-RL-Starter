# MuJoCo-RL-Starter

## Project Overview
This repository contains a custom Reinforcement Learning (RL) and testing environment for mobile robots, built upon the **MuJoCo** physics engine. The project aims to establish a complete pipeline from simulation modeling to algorithm deployment.

The architecture is designed to support:
1.  **Low-level Physics Simulation**: High-fidelity contact dynamics using MuJoCo.
2.  **RL Training Interface**: Standardized Python API for state extraction and control.
3.  **ROS2 Integration**: Sim-to-Real bridging using ROS2 nodes for planning and control.

## Tech Stack
- **OS**: Ubuntu (20.04/22.04)
- **Physics Engine**: MuJoCo
- **Language**: Python 3.x, C++ (optional for specific nodes)
- **Middleware**: ROS2 (Humble/Foxy)
- **RL Standard**: Gymnasium (OpenAI Gym compatible)
- **Version Control**: Git/GitHub

---

## 🗺️ Roadmap & Task List

### Phase 1: Environment Setup & Modeling (MuJoCo Basics)
*Goal: Establish the physics world and import robot models.*
- [ ] **Environment Initialization**
    - [ ] Install `mujoco` python bindings and visualizer.
    - [ ] Create a base project structure.
- [ ] **Robot Modeling**
    - [ ] Select/Design a mobile robot model (e.g., Quadruped or Wheeled).
    - [ ] Convert URDF models to MJCF (MuJoCo XML) format.
    - [ ] Tune physical parameters (damping, friction, armature).
- [ ] **Scene Construction**
    - [ ] Create `world.xml` with ground plane, lighting, and textures.
    - [ ] Implement capability to spawn static obstacles (geoms) dynamically.
    - [ ] Verify kinematics and dynamics in the `mujoco.viewer`.

### Phase 2: Data Interface & Gym Wrapper
*Goal: Create the API for State (Input) and Action (Output) flow.*
- [ ] **Low-Level Interface (`mujoco` bindings)**
    - [ ] Implement **State Extraction**: Read `data.qpos` (position), `data.qvel` (velocity).
    - [ ] Implement **Sensor Simulation**: Add IMU/Lidar/Camera in XML and read `data.sensordata`.
    - [ ] Implement **Collision Detection**: Parse `data.contact` to detect robot-obstacle interaction.
    - [ ] Implement **Motor Control**: Map control inputs to `data.ctrl` (torque/position actuators).
- [ ] **Gymnasium Integration**
    - [ ] Create a class inheriting from `gym.Env`.
    - [ ] Define `observation_space` (State vector) and `action_space`.
    - [ ] Implement standard `reset()` and `step()` methods.

### Phase 3: RL Algorithm Construction
*Goal: Develop the brain of the robot using the Gym interface.*
- [ ] **Task Definition**
    - [ ] Design the Reward Function (e.g., +Reward for velocity, -Penalty for collision/energy).
    - [ ] Define "Done" conditions (Termination logic).
- [ ] **Algorithm Implementation**
    - [ ] Set up an RL agent (e.g., PPO, SAC) using libraries like Stable Baselines3 or custom implementation.
    - [ ] Train the agent to navigate to random target points.
- [ ] **Visualization**
    - [ ] Implement render loop for training (headless) vs. inference (GUI).
    - [ ] Log training curves (Tensorboard/WandB).

### Phase 4: ROS2 Bridge & System Integration
*Goal: Decouple planning and control into ROS2 nodes for Sim-to-Real readiness.*
- [ ] **Bridge Node Development**
    - [ ] Create a `mujoco_ros_bridge` node.
    - [ ] **Clock Sync**: Publish `/clock` based on simulation step time (Sim Time).
    - [ ] **State Publisher**: Publish robot state to `/joint_states` and `/odom`.
    - [ ] **Command Subscriber**: Subscribe to `/cmd_vel` or joint commands and write to MuJoCo `data.ctrl`.
- [ ] **Modular Control Architecture**
    - [ ] Create a **Planner Node**: Runs the trained RL policy, outputs high-level commands.
    - [ ] Create a **Controller Node**: Handles low-level inverse kinematics or PID (optional, if not end-to-end).
- [ ] **Integration Testing**
    - [ ] Verify synchronization between MuJoCo physics step and ROS2 callbacks.
    - [ ] Visualize robot TF tree in **RViz2** alongside MuJoCo window.

---

## Quick Start
*(To be populated...)*