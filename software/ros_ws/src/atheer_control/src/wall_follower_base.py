#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class WallFollowerBase:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_pwm", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/cmd_steer", Float32, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.callback)

        self.TARGET_DIST = 0.4
        self.KP = 150.0        # Adjusted for the new -100 to 100 scale
        self.BASE_SPEED = 50.0 # 50% throttle instead of raw 180 PWM

        # ESP32 expects -100 (left) to 100 (right), 0 is center
        self.STEER_CENTER = 0.0
        self.STEER_MIN = -100.0
        self.STEER_MAX = 100.0

    def get_range(self, msg, angle_deg):
        angle_rad = math.radians(angle_deg)
        
        # Normalize angle to strictly fall between angle_min and angle_max
        while angle_rad < msg.angle_min:
            angle_rad += 2 * math.pi
        while angle_rad > msg.angle_max:
            angle_rad -= 2 * math.pi

        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        # Check bounds
        if 0 <= index < len(msg.ranges):
            r = msg.ranges[index]
            if not math.isinf(r) and not math.isnan(r):
                return r
        return None

    def compute_error(self, msg):
        raise NotImplementedError("Must implement in child class")

    def callback(self, msg):
        error = self.compute_error(msg)

        steer = self.STEER_CENTER - self.KP * error
        steer = max(self.STEER_MIN, min(self.STEER_MAX, steer))

        self.vel_pub.publish(Float32(self.BASE_SPEED))
        self.steer_pub.publish(Float32(steer))