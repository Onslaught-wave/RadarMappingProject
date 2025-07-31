
2D Environmental Mapping System

A radar‑style 2D environmental mapping project using ESP32, ultrasonic (HC‑SR04), and LiDAR sensors (TFmini Plus, VL53L0X).
The system rotates sensors with a stepper motor (28BYJ‑48) to scan the surroundings, collect real‑time distance data, and visualize the environment on a PC.
Written in C++, the project compares sensor performance and demonstrates environmental mapping for robotics and IoT applications.

Features:

Multi‑sensor integration:
Combines ultrasonic (HC‑SR04) and laser ToF (TFmini Plus, VL53L0X) distance sensors
ESP32 firmware in C++:
Controls sensors, stepper motor, and handles data acquisition
Rotating scanning platform:
Stepper motor (28BYJ‑48) with ULN2003 driver for precise angular positioning

Real‑time visualization:

Send distance data to a PC for 2D mapping and CSV/Excel export
Sensor performance evaluation:
Compare accuracy and reliability of ultrasonic vs. LiDAR sensors

 Hardware Used:

ESP32‑WROOM‑32 microcontroller
HC‑SR04 ultrasonic sensor
TFmini Plus LiDAR sensor
VL53L0X ToF LiDAR sensor
28BYJ‑48 stepper motor with ULN2003 driver
Rotating platform with sensor mount

How It Works:

The ESP32 rotates the sensor platform via the stepper motor
Distance data is collected from all sensors in real time
Data is sent over UART or Bluetooth to a PC
Desktop app visualizes the scanned environment and allows data export


Setup:

Flash the ESP32 firmware from the /esp32 folder
Wire the sensors and stepper motor as in the /hardware schematic
Run the Processing app in /processing to visualize the scan
(Optional) Export results to CSV/Excel