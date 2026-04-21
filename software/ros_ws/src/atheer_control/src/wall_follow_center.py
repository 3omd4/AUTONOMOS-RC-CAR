#!/usr/bin/env python3
from wall_follower_base import WallFollowerBase
import rospy

class CenterFollower(WallFollowerBase):

    def compute_error(self, msg):
        left = self.get_range(msg, 90)
        right = self.get_range(msg, -90)

        if left is None or right is None:
            return 0

        # center error (positive → too close to right wall)
        return (right - left)


if __name__ == "__main__":
    rospy.init_node("center_follower")
    CenterFollower()
    rospy.spin()
