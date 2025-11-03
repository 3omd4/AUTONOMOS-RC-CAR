#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

# =================== CONFIG ===================
TARGET_DIST = 0.4          # desired distance from right wall (m)
KP_DIST = 200.0         # gain for steering control
BASE_SPEED = 180.0           # desired forward speed (pwm)
SCAN_RIGHT_START = -100.0  # degrees
SCAN_RIGHT_END = -60.0     # degrees
# ==============================================


class WallFollower:
    def __init__(self):
        rospy.init_node("right_wall_follower", anonymous=True)

        # Publishers to Arduino node
        self.vel_pub = rospy.Publisher("/cmd_pwm", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/cmd_steer", Float32, queue_size=10)

        # Subscribe to LiDAR
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        rospy.loginfo("Wall follower node started.")
        rospy.spin()

    def lidar_callback(self, msg):
        # Extract right-side distances
        distances = []
        for i, r in enumerate(msg.ranges):
            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            if SCAN_RIGHT_START <= angle_deg <= SCAN_RIGHT_END and not math.isinf(r):
                distances.append(r)

        if len(distances) < 3:
            return  # not enough valid points

        # Average distance to right wall
        avg_right = sum(distances) / len(distances)
        error = avg_right - TARGET_DIST  # positive if too far from wall

        # Compute steering correction
        steer_angle = 95 + KP_DIST * error
        steer_angle = max(78, min(112, steer_angle))  # clip to servo limits

        # Publish control commands
        vel_msg = Float32()
        vel_msg.data = BASE_SPEED

        steer_msg = Float32()
        steer_msg.data = steer_angle

        self.vel_pub.publish(vel_msg)
        self.steer_pub.publish(steer_msg)

        print(f"RightDist= ",avg_right,  "| Err= ", error, "| Servo= ", steer_angle)


if __name__ == "__main__":
    try:
        WallFollower()
    except rospy.ROSInterruptException:
        pass
