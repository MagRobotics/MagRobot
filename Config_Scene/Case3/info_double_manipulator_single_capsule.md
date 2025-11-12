# Simulation Environment Parameter Configuration

**Scenario Type:** Dual Manipulator + Single Capsule Endoscope System

**Description:** This scenario simulates the cooperative magnetic control of a single capsule endoscope in a deformable stomach environment, using two robotic manipulators for enhanced manipulation capabilities.

## 1. Soft Environment Parameters

### 1.1 Stomach Model
- Environment Type: soft_fixed
- Surface Mesh File: model/environment/stomach.stl
- Volume Mesh File: model/environment/stomach.msh
- Pose: [0.27, 0.12, 0.005, 0, 0, -0.5, 0.866]
- Scale Factor: 0.035
- Mass: 1.0
- Young's Modulus: 100000
- Poisson's Ratio: 0.3
- Color: [1.0, 0.0, 0.0, 0.1]
- Boundary Indices:
  - [[0.01, 0.31, 0.95, 0.04, 0.35, 0.99],
  - [0.005, 0.125, 0.95, 0.035, 0.145, 0.99],
  - [0.01, 0.18, 0.935, 0.1, 0.28, 0.95],
  - [0.225, 0.14, 0.95, -0.03, 0.24, 0.96]]

### 1.2 Desk Model
- Environment Type: visu
- Surface Mesh File: model/environment/desk.stl
- Pose: [-0.25, 0.12, -0.235, -0.707, 0, 0, 0.707]
- Scale Factor: 0.001
- Color: [1, 1, 1, 0]

### 1.3 Body Model
- Environment Type: visu
- Surface Mesh File: model/environment/body.obj
- Pose: [-1.2, 0.12, 0.02, 0, 0, -0.707, 0.707]
- Scale Factor: 0.095
- Color: [0.93333, 0.83529, 0.82353, 0]

## 2. Capsule Parameters
- Object Type: capsule
- Pose: [0.3, 0.1, 0.02, 0, -0.3826843, 0, 0.9238792]
- Magnetic Moment: 0.1015
- Capsule Mass: 0.00289
- Buoyancy: 0.0148

## 3. Manipulator Configuration

### 3.1 Manipulator 1
- Drive Type: yidong_rob_mag
- Manipulator Base Pose: [0.27, 0.64211470775, -0.17837477746, 0, 0, 0, 1]
- Magnetic Moment: 19.5

### 3.2 Manipulator 2
- Drive Type: yidong_rob_mag
- Manipulator Base Pose: [0.27, -0.44211470775, -0.17837477746, -0.020725626204645912, 0.017713405819144545, -0.009916551514607094, 0.9995790842543524]
- Magnetic Moment: 19.5

## 4. Environment Settings
- Gravity: [0.0, 0.0, 0]
- Time Step: 0.01
- Friction Coefficient: 0.04
- Contact Distance: 0.003
- Background Color: [1, 1, 1]
- Field Display: [[0.17, 0.05, 0.160, 0.37, 0.175, 0.235, 0.025]]

## 5. Path Settings
- Path Type: yushe

## 6. Algorithm Configuration
- Algorithm Type: none

## 7. Control Parameters
- Control Type: PID
- kp1: 0.1
- ki1: 1
- kd1: 0
