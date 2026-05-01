# Navigation Logic

This document explains the basic navigation logic of the underwater robot project.

## 1. GPS Position Reading

The GPS module outputs NMEA sentences.  
The program reads GPS data in a non-blocking way and parses valid RMC sentences.

When a valid GPS fix is available, the program updates:

- Current latitude
- Current longitude
- GPS fix status
- Last valid GPS fix time

## 2. Initial Forward Direction Calibration

At startup, the robot records two GPS points:

1. Start point 1
2. Start point 2 after the robot moves forward for a short distance

The bearing from point 1 to point 2 is regarded as the robot's real initial forward direction.

This step is used to calculate the offset between the IMU yaw angle and the robot's real movement direction.

## 3. Target Bearing Calculation

The program calculates the bearing from the current GPS position to the target point.

The bearing definition is:

- 0 degrees: North
- 90 degrees: East
- 180 degrees: South
- 270 degrees: West

## 4. Heading Error Calculation

The robot compares:

- Target bearing from GPS
- Current robot heading from IMU after offset calibration

The angle error is normalized to the range of -180 degrees to +180 degrees.

Meaning:

- Positive error: the target is on the right side
- Negative error: the target is on the left side
- Small error: the robot can move forward

## 5. Motor Control Strategy

The current logic uses simple differential control:

- If the error is small, both horizontal motors move forward.
- If the target is on the right, one motor is driven to turn right.
- If the target is on the left, one motor is driven to turn left.

## 6. Depth Control

The robot reads the current depth from the depth sensor.

The target depth is compared with the current depth, and a PID controller outputs PWM signals to the vertical motors.

## 7. Current Testing Stage

The current stage focuses on:

- Indoor pool testing
- Straight movement
- Turning control
- Depth control
- Outdoor GPS simulation
- Waypoint navigation verification
