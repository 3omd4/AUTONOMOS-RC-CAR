#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32

class SerialBridge:
    def __init__(self):
        rospy.init_node("serial_bridge")

        # USB serial — typically /dev/ttyUSB0 (CP210x/CH340) or /dev/ttyACM0 (native USB)
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~baud", 115200)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            rospy.loginfo(f"Connected to ESP32 on {self.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            rospy.logerr("Check: ls /dev/ttyUSB* or ls /dev/ttyACM*")
            return

        self.steer = 0
        self.throttle = 0

        rospy.Subscriber("/cmd_steer", Float32, self.steer_cb)
        rospy.Subscriber("/cmd_pwm",   Float32, self.throttle_cb)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_command()
            rate.sleep()

    def steer_cb(self, msg):
        self.steer = int(msg.data)

    def throttle_cb(self, msg):
        self.throttle = int(msg.data)

    def send_command(self):
        cmd_str = f"CMD:{self.steer},{self.throttle}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            rospy.logwarn_throttle(2, f"Failed to send serial data: {e}")

if __name__ == "__main__":
    try:
        SerialBridge()
    except rospy.ROSInterruptException:
        pass