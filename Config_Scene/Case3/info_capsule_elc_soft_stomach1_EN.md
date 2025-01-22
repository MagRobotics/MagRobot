# Simulation Environment Parameter Configuration

This scenario simulates the navigation process of the capsule endoscope in the gastrointestinal tract. A 5-magnetic sensor array is used for pose estimation, with a sensor noise level of 4e-6.

## 1. Soft Environment Parameters
- Environment Type: soft_fixed
- Surface Mesh File: model/environment/stomach.stl
- Volume Mesh File: model/environment/stomach.msh
- Pose: [0.03, 0.25, 0.97, 0, 0.0, 0.0, 1]
- Scale Factor: 0.022
- Mass: 1.0
- Young's Modulus: 100000
- Poisson's Ratio: 0.3
- Color: [1.0, 0.0, 0.0, 0.1]
- Boundary Indices:
[[0.01, 0.31, 0.95, 0.04, 0.35, 0.99], [0.005, 0.125, 0.95, 0.035, 0.145, 0.99], [0.01, 0.18, 0.935, 0.1, 0.28, 0.95], [-0.075, 0.14, 0.95, -0.03, 0.24, 0.96]]

## 2. Capsule Parameters
- Object Type: capsule
- Pose: [0.015, 0.37, 0.97, 0.0, 0.0, 0.7071067811865475, 0.7071067811865475]
- Buoyancy: 0.0148
- Mass: 0.0085612244897
- Inertia Matrix: [6.12e-5, 0.0, 0.0, 0.0, 6.12e-5, 0.0, 0.0, 0.0, 6.12e-5]
- Magnetic Moment: 0.126

## 3. Electromagnetic Coil Configuration
8 electromagnetic coils with parameters as follows:

1. Coil 1:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.17, 0.25, 1.15, 1.0, 0.0, 0.0]
   - Number of Turns: 100

2. Coil 2:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.17, 0.25, 1.15, -1, 0.0, 0.0]
   - Number of Turns: 100

3. Coil 3:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0, 0.08, 1.15, 0, 1, 0.0]
   - Number of Turns: 100

4. Coil 4:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0, 0.42, 1.15, 0, -1, 0.0]
   - Number of Turns: 100

5. Coil 5:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.1012, 0.3512335, 1.2034, -0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 100

6. Coil 6:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.1012, 0.3512335, 1.2034, 0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 100

7. Coil 7:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.1012335, 0.1487665, 1.20342525, 0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 100

8. Coil 8:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.1012335, 0.1487665, 1.20342525, -0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 100

## 4. Sensor Configuration
1. Sensor 1:
   - Position: [-0.0076, 0.21, 0.896, 0.0, 0.0, 0.0, 1.0]
   - Noise: 4e-6

2. Sensor 2:
   - Position: [0.045, 0.28, 0.896, 0.0, 0.0, 0.0, 1.0]
   - Noise: 4e-6

3. Sensor 3:
   - Position: [-0.0076, 0.34, 0.896, 0.0, 0.0, 0.0, 1.0]
   - Noise: 4e-6

4. Sensor 4:
   - Position: [0.0976, 0.21, 0.896, 0.0, 0.0, 0.0, 1.0]
   - Noise: 4e-6

5. Sensor 5:
   - Position: [0.0976, 0.34, 0.896, 0.0, 0.0, 0.0, 1.0]
   - Noise: 4e-6

## 5. Environment Settings
- Gravity: [0.0, 0.0, -9.8]
- Force Display Scale: [0.0, 0.0]
- Time Step: 0.02
- Friction Coefficient: 0.1
- Contact Distance: 0.001
- Background Color: [1, 1, 1]
- Camera Parameters: cam=0
- Trajectory Parameters: tra=1
- Field Display: 0

## 6. Path Settings
- Path Type: yushe
- Type: Offline_Interpolation

[0.0204,0.3388,0.97,-0.0,0.0,90.0]

[0.0252,0.3142,0.97,-0.0,0.0,104.85]

[0.0306,0.2974,0.97,-0.0,0.0,118.35]

[0.0396,0.283,0.97,-0.0,0.0,127.8]

[0.0534,0.2632,0.97,-0.0,0.0,131.85]

[0.0624,0.235,0.97,-0.0,0.0,98.1]

[0.0414,0.2152,0.97,-0.0,0.0,46.8]

## 7. Algorithm Configuration
### EKF Parameters
- Algorithm Type: EKF
- EKF_Q: 6x6 Identity Matrix

[[1, 0, 0, 0, 0, 0],
[0, 1, 0, 0, 0, 0],
[0, 0, 1, 0, 0, 0],
[0, 0, 0, 1, 0, 0],
[0, 0, 0, 0, 1, 0],
[0, 0, 0, 0, 0, 1]]

- EKF_R: 15x15 Identity Matrix

[[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]

## 8. Control Parameters
- Position Control: PID
- kp: 40
- ki: 0
- kd: 40
- Attitude Control: PID
- kp2: 80000
- ki2: 0
- kd2: 6000
