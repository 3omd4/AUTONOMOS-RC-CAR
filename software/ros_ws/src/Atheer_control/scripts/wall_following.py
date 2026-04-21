#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

# =================== CONFIG ===================
TARGET_DIST = 0.4        # desired distance from right wall (m)
KP_DIST = 200.0          # steering gain
BASE_SPEED = 180.0       # forward speed (pwm)

ANGLE_RIGHT = -90        # degrees
ANGLE_FRONT_RIGHT = -45  # degrees

SERVO_CENTER = 95
SERVO_MIN = 78
SERVO_MAX = 112
# ==============================================


class WallFollower:
    def __init__(self):
        rospy.init_node("right_wall_follower", anonymous=True)

        self.vel_pub = rospy.Publisher("/cmd_pwm", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/cmd_steer", Float32, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        rospy.loginfo("Wall follower (2-ray method) started.")
        rospy.spin()

    # -------- Helper: get distance at angle --------
    def get_range(self, msg, angle_deg):
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        if 0 <= index < len(msg.ranges):
            r = msg.ranges[index]
            if not math.isinf(r) and not math.isnan(r):
                return r
        return None

    # -------- Main callback --------
    def lidar_callback(self, msg):
        a = self.get_range(msg, ANGLE_FRONT_RIGHT)
        b = self.get_range(msg, ANGLE_RIGHT)

        if a is None or b is None:
            return

        theta = math.radians(abs(ANGLE_FRONT_RIGHT - ANGLE_RIGHT))  # ~45°

        # Estimate wall angle
        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))

        # Distance perpendicular to wall
        dist = b * math.cos(alpha)

        # Control error
        error = dist - TARGET_DIST

        # Steering control
        steer = SERVO_CENTER + KP_DIST * error
        steer = max(SERVO_MIN, min(SERVO_MAX, steer))

        # Publish
        vel_msg = Float32()
        vel_msg.data = BASE_SPEED

        steer_msg = Float32()
        steer_msg.data = steer

        self.vel_pub.publish(vel_msg)
        self.steer_pub.publish(steer_msg)

        # Debug
        print(f"a={a:.2f}, b={b:.2f}, alpha={math.degrees(alpha):.2f}, "
              f"dist={dist:.2f}, err={error:.2f}, steer={steer:.2f}")


if __name__ == "__main__":
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
