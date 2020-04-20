#!/usr/bin/env python

"""
Description: Node to read msgs from the joy node and send out cmd_vel msgs that
the husky listens for

Authors: Ryan

"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Callback function to convert joy data to velocity commands
def vel_callback(msg):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	vel_msg = Twist()

	if msg.buttons[0]:
		vel_msg.linear.x = 1

	elif msg.buttons[1] == 1:
		vel_msg.linear.x = 2
		vel_msg.angular.z = 2.5
	
	pub.publish(vel_msg)


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('teleop_node', anonymous=True)

	# Subscribe to joy topic to get input from controller
	rospy.Subscriber("joy", Joy, vel_callback)

	rospy.spin()
