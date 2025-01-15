# Simulation Environment Parameter Configuration

## 1. Environment Parameters
- Environment Type: rigid_fixed
- Surface Mesh File: model/environment/bronchus.stl
- Pose: [0.0, 0.35, 0.97, 5.336750069161485e-18, 0.9961946980917454, -0.08715574274765815, 6.0999332417281e-17]
- Scale: 0.025
- Color: [1.0, 0.0, 0.0, 0.15]
- Normal Flip: 1
- Is Static: 1
- Total Mass: 0.1

## 2. Catheter Parameters
- Object Type: wire
- Pose: [-0.002, 0.52, 0.988, 0.030843564597231896, 0.030843564597231896, -0.7064337722128922, 0.7064337722128922]
- Length: 0.5
- Inner Diameter: 0.0015
- Outer Diameter: 0.002
- Young's Modulus: 100000000
- Length Density: [0.015, 0.023]
- Magnetic Moment: 0.03

## 3. Robot Configuration
- Drive Type: rob
- Robot Base Pose: [-0.5, 0.2, 0.745, 0, 0, 0, 1]
- Magnetic Moment: 1400
- Magnet Pose: [0.243067083, 0.348063424, 1.000999998844, -1.1191633e-25, 4.26471861e-09, 0.382675455, 0.923882837]
- Flag: 1
- Visualization: mag

## 4. Environment Settings
- Gravity: [0.0, 0.0, -9.8]
- Force Display Scale: [0.0, 0.0]
- Time Step: 0.03
- Friction Coefficient: 0.01
- Contact Distance: 0.0013
- Background Color: [1, 1, 1]
- Camera Parameters: cam=0
- Trajectory Parameters: tra=0
- Field Display: 0

## 5. Path Settings
- Path Type: key

## 6. Localization Algorithm Configuration
- Algorithm Type: none

## 7. Control Parameters
- Control Type: wire_openloop
- Magnetic Field Strength: 0.004828 