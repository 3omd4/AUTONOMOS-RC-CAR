# 🧩 Arduino Folder Overview

This folder contains all the **required libraries and Arduino codes** used in the *Autonomous RC Car* project.  
It is based on the **MPU6050 calibration and control framework** developed by:
- **Luis Ródenas** — for sensor calibration and bias estimation.  
- **Jeff Rowberg** — for the original [I2Cdev](https://github.com/jrowberg/i2cdevlib) and MPU6050 libraries.

---

## 📁 Folder Structure

Arduino/
│
├── Arduino_documentation.md # Additional project notes or documentation
├── ROS_control_code/ # Final code used for ROS–Arduino communication
└── libraries/
├──── I2Cdev/ # I2C communication driver
├──── MPU6050/ # MPU6050 library and calibration examples
│ ├── MPU_calibration/ # Code for calibrating sensor offsets and bias
│ └── examples/
│ ├── combined_code/ # Combined test for servo, DC motors, and encoder
│ ├── IMU_Zero/ # Example for zeroing IMU offsets
│ ├── MPU6050_DMP6/ # DMP-based IMU example using quaternion data
│ └── MPU6050_raw/ # Raw accelerometer and gyroscope data example
└──── Servo/ # Servo motor control library

---

## Key Codes & Their Purpose

| **Code Name**          | **Location** | **Purpose** |
|-------------------------|---------------|--------------|
| **MPU_calibration.ino** | `MPU6050/MPU_calibration/` | Calibrates MPU6050 bias and offset values for accurate motion data. |
| **combined_code.ino**   | `MPU6050/examples/combined_code/` | Tests servo, DC motor, and encoder control together. |
| **MPU6050_DMP6.ino**    | `MPU6050/examples/MPU6050_DMP6/` | Demonstrates using the MPU6050’s DMP (Digital Motion Processor) for orientation data (yaw, pitch, roll). |
| **MPU6050_raw.ino**     | `MPU6050/examples/MPU6050_raw/` | Reads raw accelerometer and gyroscope values for analysis or debugging. |
| **ROS_control_code.ino**| `ROS_control_code/` | Final implementation for `rosserial` communication between Jetson and Arduino, handling motors, encoders, and MPU6050. |

---

## Dependencies

Before running any of the Arduino codes, ensure you have installed the following libraries:
- **I2Cdev Library** → Handles I2C communication  
- **MPU6050 Library** → Provides functions to read and process IMU data  
- **Servo Library** → Controls servo motors  
- **rosserial_arduino** → Enables ROS 2 communication between Jetson and Arduino

You can install them by placing the folders inside your Arduino `libraries/` directory or by using the Arduino IDE Library Manager.

---

## 📍 Usage Notes

1. **Calibration First:**  
   Upload and run `MPU_calibration.ino` to get bias and offset values for your MPU6050.  
   Update these values in your final control code if needed.

2. **Testing Hardware:**  
   Use `combined_code.ino` to verify correct behavior of motors, servos, and encoders before integrating ROS.

3. **IMU Testing:**  
   - Use `MPU6050_raw.ino` to confirm sensor readings.  
   - Use `MPU6050_DMP6.ino` to view stable yaw/pitch/roll data using the onboard DMP.

4. **ROS Communication:**  
   Once everything works, upload `ROS_control_code.ino` and connect your Arduino to the Jetson using `rosserial`.

---
