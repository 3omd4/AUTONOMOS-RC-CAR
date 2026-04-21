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
        self.KP = 200
        self.BASE_SPEED = 180

        self.SERVO_CENTER = 95
        self.SERVO_MIN = 78
        self.SERVO_MAX = 112

    def get_range(self, msg, angle_deg):
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        if 0 <= index < len(msg.ranges):
            r = msg.ranges[index]
            if not math.isinf(r) and not math.isnan(r):
                return r
        return None

    def compute_error(self, msg):
        raise NotImplementedError("Must implement in child class")

    def callback(self, msg):
        error = self.compute_error(msg)

        steer = self.SERVO_CENTER + self.KP * error
        steer = max(self.SERVO_MIN, min(self.SERVO_MAX, steer))

        self.vel_pub.publish(Float32(self.BASE_SPEED))
        self.steer_pub.publish(Float32(steer))
