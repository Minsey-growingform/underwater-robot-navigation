# Underwater Robot Navigation and Control

This repository documents an ESP32-based underwater robot navigation and control project.

The project combines GPS positioning, IMU heading estimation, depth sensing, and PWM motor control to support basic waypoint navigation and underwater motion control.

## Project Background

This project is part of my undergraduate robotics practice.  
The goal is to build and test a simple navigation logic for an underwater robot:

1. Read current GPS position on the water surface.
2. Calculate the bearing from the current position to the target point.
3. Use IMU yaw data to estimate the robot's current heading.
4. Compare target bearing and robot heading.
5. Control the horizontal motors to turn or move forward.
6. Use the depth sensor and vertical motors to maintain target depth.

## Current Features

- ESP32-based main control
- GPS data parsing from NMEA RMC sentences
- IMU yaw data parsing through UART
- Depth sensor data reading through UART
- Horizontal motor control using PWM
- Vertical motor control using depth PID
- Initial forward direction calibration using two GPS points
- Target bearing calculation based on latitude and longitude
- Basic waypoint navigation logic

## Hardware Modules

| Module | Function |
|---|---|
| ESP32 | Main controller |
| GPS module | Positioning and target bearing calculation |
| IMU module | Yaw / heading angle measurement |
| Depth sensor | Real-time depth measurement |
| ESC + brushless motors | Horizontal and vertical motion control |

## Code Structure

The current main program includes the following parts:

- Depth sensor reading and zero-offset calibration
- IMU initialization and yaw parsing
- GPS non-blocking reading and RMC parsing
- Bearing calculation
- Angle error calculation
- Horizontal motor control
- Depth PID control
- Initial GPS + IMU forward direction calibration
- Main navigation loop

## AI-assisted Development

AI tools were used to help with:

- Code structure analysis
- GPS / IMU / depth sensor logic explanation
- Navigation algorithm organization
- Debugging ideas
- Technical documentation
- Project record writing

## Current Status

The robot has completed basic indoor pool testing for straight movement, turning, and depth control.  
The next stage is outdoor GPS-based navigation testing and waypoint movement verification.

## Notes

This repository is mainly used to record project learning, debugging, and development progress.
