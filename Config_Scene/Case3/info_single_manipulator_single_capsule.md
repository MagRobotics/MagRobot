# Simulation Environment Parameter Configuration

**Scenario Type:** Single Manipulator + Single Capsule Endoscope System

**Description:** This scenario simulates the magnetic navigation of a capsule endoscope in a deformable gastrointestinal tract environment, using a single robotic manipulator with magnetic actuation for precise control.

## 1. Soft Environment Parameters

### 1.1 Stomach (Deformable)
- Environment Type: soft_fixed
- Surface Mesh File: model/environment/stomach.stl
- Volume Mesh File: model/environment/stomach.msh
- Pose: [-0.21, 0, 0.95, 0, 0, 0, 1]
- Scale Factor: 0.035
- Mass: 1.0
- Young's Modulus: 100000
- Poisson's Ratio: 0.3
- Color: [1.0, 0.0, 0.0, 0.1]
- Is Static Object: Yes
- Flip Normals: Yes
- Boundary Indices:
  - [[0.01, 0.31, 0.95, 0.04, 0.35, 0.99],
  - [0.005, 0.125, 0.95, 0.035, 0.145, 0.99],
  - [0.01, 0.18, 0.935, 0.1, 0.28, 0.95],
  - [0.225, 0.14, 0.95, -0.03, 0.24, 0.96]]

### 1.2 Desk (Visual Only)
- Environment Type: visu
- Surface Mesh File: model/environment/desk.stl
- Pose: [-0.36, -0.42, 0.695, -0.5, -0.5, 0.5, 0.5]
- Scale Factor: 0.001
- Color: [1, 1, 1, 0]

### 1.3 Body (Visual Only)
- Environment Type: visu
- Surface Mesh File: model/environment/body.obj
- Pose: [-0.37, -1.4, 0.95, 0, 0, 0, 1]
- Scale Factor: 0.095
- Color: [0.93333, 0.83529, 0.82353, 0]

## 2. Capsule Parameters
- Object Type: capsule
- Pose: [-0.32, 0, 0.95, 0, 0.3826843, 0, 0.9238792]
- Buoyancy: 0.0148
- Mass: 0.0015612244897
- Inertia Matrix: [6.12e-6, 0.0, 0.0, 0.0, 6.12e-6, 0.0, 0.0, 0.0, 6.12e-6]
- Magnetic Moment: 0.126

## 3. Magnetic Manipulator Configuration
- Drive Type: yidong_rob_mag
- Manipulator Base Pose: [-0.95, 0, 0.76, 0, 0, 0, 1]
- Magnetic Moment: 26.2
- Magnet Pose: [0.243067083, 0.348063424, 1.000999998844, -1.1191633e-25, 4.26471861e-09, 0.382675455, 0.923882837]
- Flag: 1
- Visual: mag

## 4. Sensor Configuration
No sensors configured in this scenario.

## 5. Environment Settings
- Gravity: [0.0, 0.0, -9.8]
- Force Display Scale: [0.0, 0.0]
- Time Step: 0.02
- Friction Coefficient: 0.1
- Contact Distance: 0.001
- Background Color: [1, 1, 1]
- Camera Parameters: cam=0
- Trajectory Parameters: tra=1
- Field Display: [[-0.37, -0.05, 1.060, -0.17, 0.05, 1.135, 0.025]]

## 6. Path Settings
- Path Type: yushe
- Type: geshan

## 7. Algorithm Configuration
- Algorithm Type: none

## 8. Control Parameters
- Position Control: PID
- kp: 1
- ki: 0
- kd: 1
