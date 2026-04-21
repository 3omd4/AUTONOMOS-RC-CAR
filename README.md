🏎️ AUTONOMOUS RC CAR (Atheer Control)
This repository contains the software and hardware integration for an autonomous wall-following RC car. The system uses a 2D RPLidar for perception, a Jetson/Ubuntu machine running ROS 1 Melodic for path planning, and an ESP-32 microcontroller communicating via Bluetooth Classic to drive the motors and steering servo.

📋 System Prerequisites
Brain: Ubuntu machine (Jetson Nano, laptop, or Raspberry Pi) running ROS 1 Melodic.

Muscle: ESP32 Microcontroller.

Sensors/Actuators: RPLidar (A1/A2), Standard Steering Servo, DC Motor with L298N Motor Driver, MPU6050 IMU.

🛠️ Step 1: ESP32 Hardware Setup & Flashing
The ESP32 acts as the low-level hardware driver. It receives percentage-based commands (-100 to 100) from ROS and maps them to physical PWM signals.

Hardware Wiring:

Servo Motor (Steering): Signal -> GPIO 16

L298N (DC Motor): IN1 -> GPIO 25 | IN2 -> GPIO 33 | ENA -> GPIO 12

MPU6050 (IMU): SDA -> GPIO 21 | SCL -> GPIO 22

Flash the Firmware:

Open software/ESP-32/Atheer_RC.ino in the Arduino IDE.

Ensure you have the ESP32Servo library installed via the Library Manager.

Select your ESP32 board, select the COM port, and upload the code.

Open the Serial Monitor (115200 baud) to verify it boots and says [BT] Ready. Waiting for connection....

📡 Step 2: Bluetooth Pairing & Binding (Ubuntu/Jetson)
Because the ESP32 uses Bluetooth Classic SPP (Serial Port Profile), you must bind it to a Linux serial port so ROS can communicate with it like a standard USB device.

1. Pair the Device:
Open a terminal and run the Bluetooth configuration tool:

Bash
bluetoothctl
Inside the prompt, type the following commands:

Bash
scan on
# Wait until you see a device named "ESP32_Atheer" and copy its MAC address (e.g., AA:BB:CC:DD:EE:FF)
scan off
pair <YOUR_ESP32_MAC_ADDRESS>
trust <YOUR_ESP32_MAC_ADDRESS>
exit
2. Bind the RFCOMM Port:
Tell Ubuntu to create a virtual serial port linked to your ESP32. (Note: You must run this command every time you reboot the Jetson or power-cycle the ESP32).

Bash
sudo rfcomm bind 0 <YOUR_ESP32_MAC_ADDRESS>
3. Grant Permissions:
Give ROS permission to read and write to the newly created Bluetooth port:

Bash
sudo chmod 666 /dev/rfcomm0
⚙️ Step 3: ROS Workspace Setup
Compile the autonomy stack and ensure all Python scripts are executable.

Open a terminal and navigate to your ROS workspace:

Bash
cd ~/AUTONOMOS-RC-CAR/software/ros_ws
Make sure all Python nodes (including the Bluetooth bridge) have executable permissions:

Bash
chmod +x src/atheer_control/src/*.py
Build the workspace:

Bash
catkin_make
Source the setup file:

Bash
source devel/setup.bash
🚀 Step 4: Running the Autonomy Stack
With the hardware powered on and the Bluetooth port bound, you can bring up the entire system with a single launch file.

Ensure your RPLidar has USB permissions:

Bash
sudo chmod 666 /dev/ttyUSB0
Launch the LiDAR, Wall Follower, and Bluetooth Bridge simultaneously:

Bash
roslaunch atheer_control atheer_autonomy.launch
(By default, atheer_autonomy.launch is configured to run wall_follow_right.py. You can edit the launch file to switch to left-wall or center-wall following).

🐞 Common Troubleshooting
Error: [Errno 2] No such file or directory: '/dev/rfcomm0'

Cause: The Bluetooth serial port was not created.

Fix: Ensure the ESP32 is powered on, then run sudo rfcomm bind 0 <MAC_ADDRESS> and grant permissions with sudo chmod 666 /dev/rfcomm0.

Error: ModuleNotFoundError: No module named 'rospkg' or 'serial'

Cause: Python environment mismatch. ROS Melodic defaults to Python 2.7, but your scripts might be forcing Python 3 (#!/usr/bin/env python3).

Fix: Either change the top line of your Python scripts to #!/usr/bin/env python, OR install the Python 3 ROS tools (sudo apt install python3-pip && pip3 install rospkg defusedxml pyserial).

The car drives straight into the wall / doesn't turn:

Cause: Lidar coordinate frame mismatch.

Fix: Ensure the get_range function in wall_follower_base.py is correctly normalizing negative angles (e.g., -45 degrees) to the Lidar's 0 to 2π or -π to π coordinate space.

The steering is inverted (steers toward the wall instead of away):

Fix: Open wall_follower_base.py and invert the sign on your control error calculation, or open the ESP32 code and swap the SERVO_MIN and SERVO_MAX values.