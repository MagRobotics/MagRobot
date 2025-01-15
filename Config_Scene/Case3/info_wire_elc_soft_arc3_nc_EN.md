# Simulation Environment Parameter Configuration

## 1. Soft Environment Parameters
- Environment Type: soft_fixed
- Surface Mesh File: model/environment/arc.stl
- Volume Mesh File: model/environment/arc.msh
- Pose: [-0.005, 0.35, 0.97, 0, 0.707, 0.707, 0]
- Scale: 0.013
- Mass: 0.1
- Young's Modulus: 500000
- Poisson's Ratio: 0.2
- Color: [1.0, 0.0, 0.0, 0.1]
- Boundary Indices: 
[[-0.01,0.15,0.96,0.025,0.18,1.0], [-0.01,0.32,0.98,0.03,0.35,1.02], [-0.03,0.43,0.96,0.03,0.46,1.02]]

## 2. Catheter Parameters
- Object Type: wire
- Pose: [0.009, 0.15, 0.987, -0.09229595564125725, 0.09229595564125725, 0.7010573846499779, 0.7010573846499779]
- Length: 0.5
- Inner Diameter: 0.0009
- Outer Diameter: 0.0016
- Young's Modulus: 160000000
- Length Density: [0.015, 0.035]
- Magnetic Moment: 0.0247

## 3. Electromagnetic Coil Configuration
8 electromagnetic coils with parameters as follows:

1. Coil 1:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.17, 0.3, 1.15, 1.0, 0.0, 0.0]
   - Number of Turns: 100

2. Coil 2:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.17, 0.3, 1.15, -1, 0.0, 0.0]
   - Number of Turns: 100

3. Coil 3:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0, 0.13, 1.15, 0, 1, 0.0]
   - Number of Turns: 100

4. Coil 4:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0, 0.47, 1.15, 0, -1, 0.0]
   - Number of Turns: 100

5. Coil 5:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.1012, 0.4012335, 1.2034, -0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 100

6. Coil 6:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.1012, 0.4012335, 1.2034, 0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 100

7. Coil 7:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [-0.1012335, 0.1987665, 1.20342525, 0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 100

8. Coil 8:
   - Drive Type: elc_still
   - Radius: 0.05
   - Position: [0.1012335, 0.1987665, 1.20342525, -0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 100

## 4. Environment Settings
- Gravity: [0.0, 0.0, -9.8]
- Force Display Scale: [0.0, 0.0]
- Time Step: 0.005
- Friction Coefficient: 0.004
- Contact Distance: 0.001
- Background Color: [1, 1, 1]
- Camera Parameters: cam=0
- Trajectory Parameters: tra=0
- Field Display: 0

## 5. Path Settings
- Path Type: planning
- Pose: [-0.005, 0.35, 0.97, 0, 0.707, 0.707, 0]
- Scale: 0.013
- Type: centerline

## 6. Algorithm Configuration
- Algorithm Type: none

## 7. Control Parameters
- Position Control: PID
- kp: 150
- ki: 0.2
- kd: 0
- Attitude Control: PID
- kp2: 10
- ki2: 0.0
- kd2: 0.0 