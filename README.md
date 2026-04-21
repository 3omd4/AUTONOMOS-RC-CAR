🏎️ AUTONOMOUS RC CAR (Atheer Control)
This repository contains the software and hardware integration for an autonomous RC car. The system uses a 2D LiDAR for perception, a Jetson/Ubuntu machine running ROS 1 for path planning (wall-following), and an ESP-32 microcontroller communicating via Bluetooth Classic to drive the motors and servo.

🛠️ System Architecture
Perception: RPLidar node publishes /scan data to ROS.

Control: Python wall-following nodes (wall_follow_right.py, etc.) calculate distance and error, publishing scaling percentages (-100 to 100) to /cmd_steer and /cmd_pwm.

Bridge: A serial bridge script formats the ROS topic data into strings and sends them to the ESP32 over Bluetooth.

Hardware (ESP32): Receives the string CMD:<steer>,<throttle>, calculates the exact raw servo angles and L298N motor driver PWM, and applies the physical outputs. It also reads an MPU6050 IMU to estimate speed and yaw.

📦 Step 1: ESP32 Hardware Setup
Wiring:

Servo Motor (Steering): Signal -> GPIO 16

L298N (DC Motor): IN1 -> GPIO 25, IN2 -> GPIO 33, ENA -> GPIO 12

MPU6050 (IMU): SDA -> GPIO 21, SCL -> GPIO 22

Flash the ESP32:

Open software/ESP-32/Atheer_RC.ino in the Arduino IDE.

Ensure you have the ESP32Servo library installed.

Select your ESP32 board and flash the code.

Open the Serial Monitor at 115200 baud. You should see [ESP32_Atheer] Booting... and [BT] Ready. Waiting for connection....

💻 Step 2: Bluetooth Pairing (Ubuntu/Jetson)
Because the ESP32 uses Bluetooth Classic SPP (Serial Port Profile), you must bind it to a Linux serial port so ROS can write to it like a USB cable.

Turn on the ESP32.

Go to your Ubuntu Bluetooth settings and pair with the device named ESP32_Atheer.

Find the MAC address of the ESP32 (you can find this in the Bluetooth settings, format XX:XX:XX:XX:XX:XX).

Bind the Bluetooth device to a serial port by opening a terminal and running:

Bash
sudo rfcomm bind 0 <YOUR_ESP32_MAC_ADDRESS>
This creates a serial port at /dev/rfcomm0.

⚙️ Step 3: ROS Workspace Setup
Open a terminal and navigate to your ROS workspace:

Bash
cd software/ros_ws
Make sure all your Python scripts are executable:

Bash
chmod +x src/atheer_control/src/*.py
Build the workspace:

Bash
catkin_make
Source the setup file:

Bash
source devel/setup.bash
(Tip: Add source ~/AUTONOMOS-RC-CAR/software/ros_ws/devel/setup.bash to your ~/.bashrc so you don't have to run it every time).

🚀 Step 4: Running the Autonomy Stack
You need to run four separate processes. Open a new terminal tab for each (remembering to run source devel/setup.bash in each new tab).

Terminal 1: Start ROS Master

Bash
roscore
Terminal 2: Start the LiDAR

Bash
# Provide permissions to the USB port first
sudo chmod 666 /dev/ttyUSB0
rosrun rplidar_ros rplidarNode
(Note: If you have a launch file for your lidar, use roslaunch rplidar_ros rplidar.launch instead).

Terminal 3: Start the Bluetooth Bridge
Make sure you created the bt_bridge.py script we discussed previously to translate ROS topics into the string format the ESP32 expects.

Bash
rosrun atheer_control bt_bridge.py _port:=/dev/rfcomm0
Terminal 4: Start the Wall Follower Algorithm
Place the car on the track next to the right wall, then run:

Bash
rosrun atheer_control wall_follow_right.py
🐞 Troubleshooting
Car drives straight into the wall / Doesn't turn: Check that the requested lidar angle in your Python code matches the RPLidar's frame. If the Lidar publishes 0 to 2π, ensure your Python node isn't requesting -45 degrees without normalizing it first.

Bluetooth Bridge fails to connect: Ensure you ran the sudo rfcomm bind 0 command. If it says "Device or resource busy", run sudo rfcomm release 0 and try again.

Car steering is inverted: In wall_follower_base.py, swap the signs on your steering output, or on the ESP32, swap SERVO_MIN and SERVO_MAX inside applySteeringCommand().