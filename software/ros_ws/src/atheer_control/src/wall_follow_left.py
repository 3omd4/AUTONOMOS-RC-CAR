#!/usr/bin/env python3
from wall_follower_base import WallFollowerBase
import rospy
import math

class LeftWallFollower(WallFollowerBase):

    def compute_error(self, msg):
        a = self.get_range(msg, 45)
        b = self.get_range(msg, 90)

        if a is None or b is None:
            return 0

        theta = math.radians(45)
        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        dist = b * math.cos(alpha)

        return dist - self.TARGET_DIST


if __name__ == "__main__":
    rospy.init_node("left_wall_follower")
    LeftWallFollower()
    rospy.spin()
