#!/usr/bin/env python
# wall_follower.py
# Subscribes to sensor_msgs/LaserScan (typically /scan from rplidar_ros).
# Computes lateral distance to the right wall and issues steering + speed commands.

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import time

class WallFollower(object):
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=False)

        # Parameters
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.target_dist = rospy.get_param('~target_distance', 0.5)  # meters
        self.kp = rospy.get_param('~kp', 1.4)
        self.kd = rospy.get_param('~kd', 0.2)
        self.max_speed = rospy.get_param('~max_speed', 0.8)  # m/s
        self.min_speed = rospy.get_param('~min_speed', 0.1)
        self.steer_gain = rospy.get_param('~steer_gain', 25.0)  # scale controller output to servo offset
        self.servo_center = rospy.get_param('~servo_center', 90.0)
        self.servo_min = rospy.get_param('~servo_min', 78.0)
        self.servo_max = rospy.get_param('~servo_max', 102.0)
        self.sector_width_deg = rospy.get_param('~sector_width_deg', 25.0)  # degrees around right side to sample

        # topics to publish commands (these are internal; hardware_driver forwards them to Arduino)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/wall_cmd_vel')
        self.cmd_steer_topic = rospy.get_param('~cmd_steer_topic', '/wall_cmd_steer')

        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Float32, queue_size=1)
        self.pub_cmd_steer = rospy.Publisher(self.cmd_steer_topic, Float32, queue_size=1)

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        self.last_error = 0.0
        self.last_time = time.time()

        rospy.loginfo("wall_follower started. Listening to %s", self.scan_topic)

    def scan_callback(self, msg):
        # compute indices for the right side (-90 degrees)
        angle_min = msg.angle_min    # radians
        angle_inc = msg.angle_increment
        ranges = msg.ranges
        if len(ranges) == 0:
            return

        # target angle for right side is -pi/2 (i.e., -90 deg). Use sector +/- sector_width/2
        center_angle = -math.pi / 2.0
        half_width = math.radians(self.sector_width_deg) / 2.0
        start_angle = center_angle - half_width
        end_angle = center_angle + half_width

        # map to indices
        start_idx = int(max(0, math.floor((start_angle - angle_min) / angle_inc)))
        end_idx = int(min(len(ranges) - 1, math.ceil((end_angle - angle_min) / angle_inc)))
        if end_idx <= start_idx:
            # fallback: pick index closest to center_angle
            center_idx = int(round((center_angle - angle_min) / angle_inc))
            start_idx = max(0, center_idx - 2)
            end_idx = min(len(ranges)-1, center_idx + 2)

        # collect valid ranges
        vals = []
        for i in range(start_idx, end_idx + 1):
            r = ranges[i]
            if r and not math.isinf(r) and not math.isnan(r) and r > 0.01:
                vals.append(r)

        if len(vals) == 0:
            # no valid readings in sector. stop or keep previous
            rospy.logwarn_throttle(5, "No valid LIDAR readings on right sector.")
            return

        # robust distance: median-like via sorting
        vals.sort()
        median_dist = vals[len(vals)//2]

        # control
        now = time.time()
        dt = now - self.last_time if now != self.last_time else 1e-3
        error = self.target_dist - median_dist
        derivative = (error - self.last_error) / dt

        steer_command_raw = (self.kp * error) + (self.kd * derivative)
        # map to servo offset and clamp
        servo_offset = steer_command_raw * self.steer_gain
        servo_angle = self.servo_center + servo_offset
        servo_angle = max(self.servo_min, min(self.servo_max, servo_angle))

        # speed scaling: reduce speed when steering magnitude large
        steer_frac = abs(servo_offset) / (self.servo_max - self.servo_center)
        steer_frac = max(0.0, min(1.0, steer_frac))
        speed = self.max_speed * (1.0 - 0.75 * steer_frac)
        speed = max(self.min_speed, min(self.max_speed, speed))

        # publish
        self.pub_cmd_steer.publish(Float32(servo_angle))
        self.pub_cmd_vel.publish(Float32(speed))

        # update state
        self.last_error = error
        self.last_time = now

if __name__ == '__main__':
    try:
        wf = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
