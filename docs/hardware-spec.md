# ScarGo Hardware Specification

## Status

This document captures the current agreed hardware and mechanical definition for the ScarGo quadruped project. Values may be tuned during bring-up, but the overall architecture is expected to remain stable.

## Platform Summary

- Robot type: quadruped robot dog
- Total degrees of freedom: 12
- Legs: 4
- Joints per leg: 3
- Main controller: ESP32-S3
- Firmware framework: ESP-IDF
- Servo controller: PCA9685
- IMU: MPU6050
- Display: SSD1306 OLED, 128x64
- RC receiver: ELRS 2.4G module on UART

## Leg Indexing

Leg numbering is defined from the robot's perspective:

- Leg 0: front-right
- Leg 1: front-left
- Leg 2: rear-right
- Leg 3: rear-left

Joint numbering on each leg is defined from the body outward:

- Joint 0: shoulder
- Joint 1: thigh
- Joint 2: calf

## Body Coordinate System

The body coordinate origin is the intersection point of the diagonals of the rectangle formed by the four shoulder joint centers.

Axis directions:

- +Y: forward
- +X: left
- +Z: upward

Each leg also has a local coordinate frame whose origin is the center of joint 0. The leg frame is related to the body frame by translation in X and Y.

## Body Geometry

Units are millimeters unless noted otherwise.

- bodyWidth = 100
- bodyLength = 150
- La = 40
- Lb = 100
- Lc = 100

Definitions:

- bodyWidth: left-right distance between the shoulder joint centers
- bodyLength: front-rear distance between the shoulder joint centers
- La: shoulder link length
- Lb: thigh link length
- Lc: calf link length from the knee axis to the foot end

## Zero Pose Definition

All servos are mechanically installed with the servo command angle at 90 degrees as the installation zero reference.

Nominal zero-pose geometry:

- The shoulder link is aligned with the body plane and points outward from the body.
- The thigh points downward.
- The calf is perpendicular to the thigh and points forward.

This is the current working geometric zero reference for software definitions.

## Leg Mechanical Structure

### Joint 0: Shoulder

- Joint 0 is called the shoulder.
- Its rotation axis is aligned with the Y direction.
- The shoulder link extends outward from the body.
- Left and right shoulders are installed in mirrored positions.
- Front and rear shoulders are also mirrored across the body.

Shoulder direction note:

- When the shoulder servo angle increases, leg 1 front-left and leg 2 rear-right move outward away from the body.
- Under the same servo-positive convention, leg 0 front-right and leg 3 rear-left move in the opposite lateral direction.
- Final implementation should still use a per-leg sign table for software clarity.

### Joint 1: Thigh

- Joint 1 is called the thigh.
- The thigh servo directly drives the thigh link.
- The thigh servo output angle is the thigh joint angle, up to leg-specific sign and offset mapping.
- The thigh rotation plane is perpendicular to the shoulder rotation axis.

### Joint 2: Calf

- Joint 2 is called the calf.
- The calf is not driven directly by a servo placed on the knee axis.
- Instead, the calf servo is mounted near the thigh servo.
- Motion is transmitted through a horn, linkage, rotating rocker plate, and a second linkage to the calf short arm.
- This creates a tandem servo transmission that keeps mass near the hip and reduces distal inertia.

### Tandem Calf Transmission

The current understanding of the calf mechanism is:

- The calf servo drives a 25T metal horn.
- Horn length is 26 mm.
- The horn drives a linkage.
- That linkage drives a rotating plate or rocker near the thigh root.
- The rotating plate has two linkage holes.
- The two holes are 90 degrees apart around the rocker center.
- The distance from the rocker center to either hole is equal.
- That radius is also 26 mm.
- A second linkage connects the rocker to a short arm on the calf.
- The short arm on the calf is also 26 mm from the knee axis.
- The knee axis to foot distance is 100 mm.

Software implication:

- The calf joint angle is not treated as a direct servo angle.
- It is a derived joint angle that depends on the relative motion of the thigh servo and calf servo, plus leg-specific installation offsets.
- A current working hypothesis is that the calf joint angle depends on a relation of the form:
  calf_joint_angle = thigh_servo_angle - calf_servo_angle plus a leg-specific 90 degree offset
- The exact sign convention still needs to be confirmed by calibration and test.

### Left-Front Leg Tandem Angle Relation

For the front-left leg as the current reference case:

- If the thigh servo is held fixed, calf servo rotation and calf extension follow a 1:1 positive relation.
- If the calf servo is held fixed, thigh servo rotation changes the thigh-calf included angle with a 1:1 negative relation.
- When the calf servo angle increases, the calf becomes more extended.
- When the thigh servo angle increases, the thigh swings backward.

Current working interpretation:

- The thigh-calf included angle for the front-left leg is modeled as a differential relation between calf servo angle and thigh servo angle.
- A working form is:
  front_left_knee_angle = calf_servo_angle - thigh_servo_angle + calibrated_offset
- The calibrated offset must absorb the difference between the kinematic knee-angle definition and the 90 degree mechanical installation mid-pose.

## Kinematic Angle vs Servo Installation Angle

A critical distinction must be maintained between the mechanical installation reference and the angles used by inverse kinematics.

Mechanical installation reference:

- All servos are installed with a servo command angle of 90 degrees.
- At this installation reference, the shoulder is parallel to the body plane and points outward.
- The thigh points downward.
- The calf is perpendicular to the thigh and points forward.

Kinematic modeling reference:

- In inverse kinematics, the calf angle is often defined from the included angle between the calf link and the extended line of the thigh link.
- This mathematical definition does not match the physical 90 degree installation posture of the leg.
- Because the servos are 180 degree servos and the mechanism has practical motion constraints, the hardware was intentionally installed around the current 90 degree mechanical mid-pose instead of a fully straight thigh-calf alignment.

Software implication:

- The output of inverse kinematics must not be sent directly to the servos.
- A conversion layer is required between kinematic joint angles and servo command angles.
- This conversion layer must account for:
  - installation zero offsets
  - mirrored leg sign differences
  - the tandem calf transmission relation
  - the angle offset between the kinematic calf definition and the physical servo installation mid-pose

Working rule:

- Inverse kinematics should solve for mathematically defined joint angles first.
- Those joint angles must then be mapped into calibrated servo command angles before output.

## Rotation Convention

Current convention notes:

- Body coordinate definitions use the right-hand rule.
- Left and right side rotations are intended to follow the right-hand rule with the thumb aligned to the positive axis direction.
- Servos are currently described with CCW as positive at the actuator level.
- Final software implementation will require per-leg and per-joint sign tables to map servo rotation into mathematical joint rotation.

## Actuators

- Servo count: 12
- Servo type: 180 degree metal gear servo
- Rated class: 35 kg servo
- Servo controller: PCA9685 over I2C_0

## Electronics Architecture

### Main MCU

- Module: ESP32-S3 Super Mini
- Application processor for all control logic
- The project uses only the 18 front-side pins on the module
- Back-side pins 14 to 38 are not used in the current design

### Control Responsibilities on ESP32-S3

The ESP32-S3 is expected to handle all of the following:

- kinematics
- attitude estimation
- RC signal processing
- IMU acquisition
- balance control
- PID loops
- display UI
- system state management

## Bus and Peripheral Allocation

### I2C

Two I2C buses are required.

I2C_0:

- SDA0: GPIO13
- SCL0: GPIO12
- Device: PCA9685 servo controller

I2C_1:

- SDA1: GPIO11
- SCL1: GPIO10
- Devices:
  - MPU6050
  - SSD1306 OLED display

### UART

Three UARTs are reserved.

COM0:

- USB-connected RX/TX
- Used for firmware flashing and serial debug

COM1:

- RX1: GPIO9
- TX1: GPIO8
- Used for ELRS 2.4G receiver input

COM2:

- RX2: GPIO1
- TX2: GPIO2
- Reserved for another chip
- Not currently in scope for the first implementation

### Other GPIO

- GPIO4: fan PWM control
- GPIO5: fan tachometer input
- GPIO6: K1 button
- GPIO7: K2 button
- GPIO3: buzzer control through a transistor to a 5V buzzer

## First-Phase Software Abstractions

This hardware definition suggests the following software modules:

- board support and pin map
- PCA9685 servo output driver
- joint and leg calibration tables
- IMU driver and attitude estimation
- ELRS receiver parser
- OLED UI and button input
- fan and buzzer drivers
- leg kinematics and geometry model
- balance and gait control

## Open Items To Finalize

The following items are intentionally left open and should be resolved during calibration and early bring-up:

- exact sign convention for each joint on each leg
- exact formula mapping servo angles to joint angles
- exact calf transmission model and whether the simplified angle relation is sufficient
- exact neutral pulse widths and mechanical limits for all 12 servos
- left/right and front/rear mirrored offset tables
- final body dimensions if small mechanical tuning occurs

## Notes

This document is meant to be revised frequently. When mechanical understanding improves, this file should be updated before kinematics formulas are locked into code.
