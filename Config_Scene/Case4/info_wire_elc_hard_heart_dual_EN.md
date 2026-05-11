# Simulation Environment Parameter Configuration

**Scenario Type:** Continuum Ablation Catheter + Stationary Eight-Coil Electromagnetic Array + Rigid Heart (Cardiac Electrophysiology)

**Description:** This scenario simulates magnetic navigation of a flexible ablation catheter in a CT-reconstructed rigid heart model for cardiac radiofrequency ablation. The catheter is delivered from the inferior vena cava through the right atrium and into the left atrium via transseptal puncture, then reaches the four pulmonary vein ostia (LSPV, RSPV, LIPV, RIPV) to perform rotational ablation. A stationary array of eight electromagnetic coils provides contactless actuation. In the dual-magnet configuration, two permanent magnets with different magnetization directions are embedded along the catheter: closed-loop lateral position control anchors the sub-distal magnet as a "virtual pivot" for stable structural support, while the distal tip remains under open-loop teleoperated orientation control.

## 1. Rigid Environment Parameters

### 1.1 Heart (Rigid Stationary)
- Environment Type: hard_fixed
- Surface Mesh File: model/environment/heart.stl
- Pose: [-0.00, -0.25, -0.03, -0.707, 0, 0, 0.707]
- Mass: 0.0
- Color: [1.0, 0.33529, 0.32353, 0.13]
- Is Static Object: Yes
- Flip Normals: Yes

## 2. Catheter Parameters
- Object Type: wire
- Pose (Initial): [-0.02, -0.08, 0.028, 0.12278780396897283, -0.12278780396897283, 0.6963642403200189, 0.6963642403200189]
- Body Length: 0.5
- Tip Length: 0.075
- Outer Diameter: 0.0013
- Inner Diameter: 0.001
- Young's Modulus (Body): 80000000
- Young's Modulus (Tip): 200000000
- Body Element Count: 30
- Tip Element Count: 4
- Visualization Node Count: 600
- Color: [0.2, 0.8, 1.0, 1.0]

### 2.1 Embedded Magnets (Dual-Magnet Configuration)
1. Distal-tip magnet (open-loop teleoperated orientation control):
   - Mounted Position: catheter distal tip (mesh node 33)
   - Magnetic Moment: 0.04
   - Magnetization Direction: deflected 15 deg from the catheter axis

2. Sub-distal magnet (closed-loop lateral position control / virtual pivot):
   - Mounted Position: sub-distal segment (mesh node 32, ~5 mm proximal to tip)
   - Magnetic Moment: 0.04
   - Magnetization Direction: coaxial with the catheter axis (different from the tip magnet)

## 3. Electromagnetic Coil Configuration
8 stationary electromagnetic coils with parameters as follows:

1. Coil 1 (-X axis):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [-0.17, 0.0, 0.25, 1.0, 0.0, 0.0]
   - Number of Turns: 2000

2. Coil 2 (+X axis):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [0.17, 0.0, 0.25, -1.0, 0.0, 0.0]
   - Number of Turns: 2000

3. Coil 3 (-Y axis):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [0.0, -0.17, 0.25, 0.0, 1.0, 0.0]
   - Number of Turns: 2000

4. Coil 4 (+Y axis):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [0.0, 0.17, 0.25, 0.0, -1.0, 0.0]
   - Number of Turns: 2000

5. Coil 5 (+X +Y, inclined):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [0.1012, 0.1012335, 0.25, -0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 2000

6. Coil 6 (-X +Y, inclined):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [-0.1012, 0.1012335, 0.25, 0.62469505, -0.62469505, -0.46852129]
   - Number of Turns: 2000

7. Coil 7 (-X -Y, inclined):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [-0.1012335, -0.1, 0.25, 0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 2000

8. Coil 8 (+X -Y, inclined):
   - Drive Type: elc_still
   - Radius: 0.1
   - Position: [0.1012335, -0.1, 0.25, -0.62469505, 0.62469505, -0.46852129]
   - Number of Turns: 2000

## 4. Sensor Configuration
No sensors configured in this scenario (closed-loop position control of the sub-distal magnet uses the SOFA mechanical state directly).

## 5. Environment Settings
- Gravity: [0.0, 0.0, 0.0]
- Force Display Scale: [0.0, 0.0]
- Time Step: 0.01
- Friction Coefficient: 0.02
- Contact Distance: 0.001
- Alarm Distance: 0.0015
- Background Color: [1, 1, 1]
- Camera Parameters: cam=1 (tip-follow camera)
- Trajectory Parameters: tra=0
- Field Display: 0

## 6. Path Settings
- Path Type: keyboard / planning
- Pose: [-0.005, 0.35, 0.97, 0, 0.707, 0.707, 0]
- Scale: 0.013
- Type: centerline (IVC -> RA -> transseptal -> LA -> 4 pulmonary vein ostia: LSPV / RSPV / LIPV / RIPV)
- Centerline File: centerline.txt

## 7. Algorithm Configuration
- Algorithm Type: none (no magnetic tracking; pose obtained directly from SOFA mechanical state)
- Inverse-Magnetics Solver: back_analytic_force_force (maps two-point desired magnetic forces to eight coil currents)

## 8. Control Parameters
### 8.1 Distal-Tip Orientation Control (open-loop teleoperated)
- Control Mode: open-loop magnetic-field direction (user keyboard input via I/J/K/L)
- Applied Quantity: torque = m_tip x B_desired
- Reference Field Magnitude: 0.0060 T

### 8.2 Sub-Distal Lateral Position Control (closed-loop, virtual pivot)
- Control Mode: PID position control on the sub-distal magnet
- Anchor Target Position: [0.0085, -0.026, 0.016]
- Engage Switch: keyboard key `T` (toggles between engage / release)
- PID Parameters:
  - kp: 200
  - ki: 50.9
  - kd: 0.0
  - dt: 0.05
  - Coefficient (force-to-output scaling): 1.53e-3
