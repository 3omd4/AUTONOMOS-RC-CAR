#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32

class BluetoothBridge:
    def __init__(self):
        rospy.init_node("bluetooth_bridge")
        
        # On Linux/Ubuntu, Bluetooth Classic SPP binds to /dev/rfcomm0
        self.port = rospy.get_param("~port", "/dev/rfcomm0")
        self.baud = rospy.get_param("~baud", 115200)
        
        try:
            # Requires pyserial: pip install pyserial
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            rospy.loginfo(f"Connected to ESP32 on {self.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to Bluetooth: {e}")
            rospy.logerr("Did you run: sudo rfcomm bind 0 <ESP32_MAC_ADDRESS> ?")
            return

        self.steer = 0
        self.throttle = 0

        rospy.Subscriber("/cmd_steer", Float32, self.steer_cb)
        rospy.Subscriber("/cmd_pwm", Float32, self.throttle_cb)

        # Run at 10Hz so we don't overflow the ESP32's Bluetooth buffer
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_command()
            rate.sleep()

    def steer_cb(self, msg):
        self.steer = int(msg.data)

    def throttle_cb(self, msg):
        self.throttle = int(msg.data)

    def send_command(self):
        # Format string exactly as ESP32's decodeCommand() expects
        cmd_str = f"CMD:{self.steer},{self.throttle}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            rospy.logwarn_throttle(2, f"Failed to send BT data: {e}")

if __name__ == "__main__":
    try:
        BluetoothBridge()
    except rospy.ROSInterruptException:
        pass