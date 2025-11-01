#!/usr/bin/env python
# hardware_driver.py
# Forwards wall-following commands to the Arduino rosserial topics.
# Also republishes Arduino Velocity to an easier-to-use topic.

import rospy
from std_msgs.msg import Float32

class HardwareDriver(object):
    def __init__(self):
        rospy.init_node('hardware_driver', anonymous=False)

        # Topics from wall_follower -> this node
        self.in_cmd_vel_topic = rospy.get_param('~in_cmd_vel_topic', '/wall_cmd_vel')
        self.in_cmd_steer_topic = rospy.get_param('~in_cmd_steer_topic', '/wall_cmd_steer')

        # Arduino topics (what rosserial/arduino expects)
        self.arduino_cmd_vel_topic = rospy.get_param('~arduino_cmd_vel_topic', '/cmd_vel')
        self.arduino_cmd_steer_topic = rospy.get_param('~arduino_cmd_steer_topic', '/cmd_steering')
        self.arduino_velocity_topic = rospy.get_param('~arduino_velocity_topic', '/Velocity')  # published by Arduino

        # Publishers to Arduino
        self.pub_cmd_vel = rospy.Publisher(self.arduino_cmd_vel_topic, Float32, queue_size=1)
        self.pub_cmd_steer = rospy.Publisher(self.arduino_cmd_steer_topic, Float32, queue_size=1)

        # Republish Arduino velocity on a normalized topic
        self.pub_current_velocity = rospy.Publisher('/current_velocity', Float32, queue_size=1)

        # Subscribers from wall_follower
        rospy.Subscriber(self.in_cmd_vel_topic, Float32, self.cb_in_cmd_vel)
        rospy.Subscriber(self.in_cmd_steer_topic, Float32, self.cb_in_cmd_steer)

        # Subscriber from Arduino
        rospy.Subscriber(self.arduino_velocity_topic, Float32, self.cb_arduino_velocity)

        rospy.loginfo("hardware_driver ready. Forwarding %s -> %s and %s -> %s",
                      self.in_cmd_vel_topic, self.arduino_cmd_vel_topic,
                      self.in_cmd_steer_topic, self.arduino_cmd_steer_topic)

    def cb_in_cmd_vel(self, msg):
        # forward directly to Arduino
        self.pub_cmd_vel.publish(msg)

    def cb_in_cmd_steer(self, msg):
        # forward steering as-is
        self.pub_cmd_steer.publish(msg)

    def cb_arduino_velocity(self, msg):
        # republish Arduino velocity under a consistent topic for other nodes
        self.pub_current_velocity.publish(msg)

if __name__ == '__main__':
    try:
        node = HardwareDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
