#!/usr/bin/env python3
from wall_follower_base import WallFollowerBase
import rospy
import math

class CenterFollower(WallFollowerBase):

    def compute_error(self, msg):
        # Local flags and variables
        L_flag = True
        R_flag = True
        L_dist = 0.0
        R_dist = 0.0

        # ================= LEFT READINGS =================
        a_L = self.get_range(msg, 45)
        b_L = self.get_range(msg, 90)

        if a_L is None or b_L is None:
            L_flag = False
        else:
            theta = math.radians(45) # 90 - 45
            alpha = math.atan((a_L * math.cos(theta) - b_L) / (a_L * math.sin(theta)))
            L_dist = b_L * math.cos(alpha)

        # ================= RIGHT READINGS =================
        a_R = self.get_range(msg, -45)
        b_R = self.get_range(msg, -90)

        if a_R is None or b_R is None:
            R_flag = False
        else:
            theta = math.radians(45) # abs(-90 - -45)
            alpha = math.atan((a_R * math.cos(theta) - b_R) / (a_R * math.sin(theta)))
            R_dist = b_R * math.cos(alpha)

        # ================= CALCULATE ERROR =================
        
        # Scenario 1: Both walls are visible (Follow Center)
        if R_flag and L_flag:
            # If L_dist > R_dist (closer to right), error is positive -> steer is negative (left)
            # If L_dist < R_dist (closer to left), error is negative -> steer is positive (right)
            return L_dist - R_dist
        
        # Scenario 2: Only Left wall is visible (Follow Left Wall at TARGET_DIST)
        elif L_flag:
            return L_dist - self.TARGET_DIST
            
        # Scenario 3: Only Right wall is visible (Follow Right Wall at TARGET_DIST)
        elif R_flag:  
            # Polarity is flipped here because right wall is negative Y axis
            return self.TARGET_DIST - R_dist
        
        # Scenario 4: No walls visible
        return 0.0

if __name__ == "__main__":
    try:
        rospy.init_node("center_follower")
        CenterFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass