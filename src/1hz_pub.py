#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("one_hz_pub", anonymous=True)

    pub = rospy.Publisher("one_hz", Int16, queue_size=1)

    # Set rate to 1 hz
    r = rospy.Rate(1)

    # Loop until roscore is closed
    while(not rospy.is_shutdown()):
        num = Int16()
        num.data = 1

        pub.publish(num)

        # Sleep to maintain 1 hz rate
        r.sleep()
