# Hardware Configuration

This document records the main hardware modules and pin configuration used in the underwater robot project.

## Main Controller

- Board: ESP32
- Development framework: Arduino-style ESP32 program

## Depth Sensor

| Item | Value |
|---|---|
| Serial port | Serial1 |
| RX pin | GPIO40 |
| TX pin | GPIO39 |
| Baud rate | 115200 |

The depth sensor outputs text data containing depth information.  
The program parses lines containing `Depth:` and extracts the depth value in meters.

## IMU Module

| Item | Value |
|---|---|
| Serial port | Serial2 |
| RX pin | GPIO2 |
| TX pin | GPIO1 |
| Baud rate | 115200 |

The IMU is used to obtain the current yaw angle of the robot.  
The program sends logging commands to the IMU and parses yaw data from binary packets.

## GPS Module

| Item | Value |
|---|---|
| Serial port | GPS_Serial |
| RX pin | GPIO13 |
| TX pin | GPIO14 |
| Baud rate | 9600 |

The GPS module outputs NMEA sentences.  
The program currently parses `$GNRMC` and `$GPRMC` sentences to obtain latitude and longitude.

## Horizontal Motors

| Motor | GPIO | LEDC Channel |
|---|---|---|
| Horizontal motor 1 | GPIO41 | Channel 0 |
| Horizontal motor 2 | GPIO42 | Channel 1 |

The horizontal motors are used for forward movement and differential turning.

## Vertical Motors

| Motor | GPIO | LEDC Channel |
|---|---|---|
| Vertical motor 1 | GPIO15 | Channel 2 |
| Vertical motor 2 | GPIO16 | Channel 3 |

The vertical motors are used for depth control.

## PWM Setting

| Item | Value |
|---|---|
| Frequency | 50 Hz |
| Resolution | 14 bit |
| Stop pulse width | 1500 us |
| Limited control range | 1300 us - 1700 us |

## Safety Notes

Before testing in water:

- Confirm that all ESCs are initialized correctly.
- Confirm that the motor direction is correct.
- Test without propellers when debugging PWM output.
- Keep the robot fixed during the first power-on test.
- Avoid uploading private locations, private test sites, or sensitive project documents directly to the public repository.
