#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int16
import copy

class goalRefresher(object):
    # Initialization function 
    def __init__(self):
        # Variable to store the latest postion message from AMCL
        self.current_pos = PoseWithCovarianceStamped()

        # Variable to store the latest goal for move_base
        self.current_goal = PoseStamped()

        # Publisher for the move_base simple goal topic 
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # Tolerance for checking between position and goal
        self.tol = .25

    # Callback function that updates the current position with the latest message from AMCL
    def posCallback(self, pos_msg):
        self.current_pos = copy.deepcopy(pos_msg)

        # print "Position:"
        # print self.current_pos

    # Callback function that updates the current goal with the latest message from move_base
    def goalCallback(self, goal_msg):
        self.current_goal = copy.deepcopy(goal_msg)

        # print "Goal:"
        # print self.current_goal

    # Checks if the current position and current goal are the same
    # If they are it publishes another goal in the same place in order to update
    # the positions of the social cost fields of the people in the simulation 
    def refresher(self, msg):
        # Check if pose is within tolerance
        if abs(self.current_pos.pose.pose.position.x - self.current_goal.pose.position.x) > self.tol:
            return
        if abs(self.current_pos.pose.pose.position.y - self.current_goal.pose.position.y) > self.tol:
            return
        if abs(self.current_pos.pose.pose.position.z - self.current_goal.pose.position.z) > self.tol:
            return
        if abs(self.current_pos.pose.pose.orientation.x - self.current_goal.pose.orientation.x) > self.tol:
            return
        if abs(self.current_pos.pose.pose.orientation.y - self.current_goal.pose.orientation.y) > self.tol:
            return
        if abs(self.current_pos.pose.pose.orientation.z - self.current_goal.pose.orientation.z) > self.tol:
            return
        if abs(self.current_pos.pose.pose.orientation.w - self.current_goal.pose.orientation.w) > self.tol:
            return

        # Create new goal message to publish
        goal = PoseStamped()

        # Populate header values
        goal.header.seq = self.current_goal.header.seq + 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.current_goal.header.frame_id

        # Populate pose values with old pose values
        goal.pose = copy.deepcopy(self.current_goal.pose)

        print "Publishing new goal"

        self.goal_pub.publish(goal)
        


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("goal_resfresher", anonymous=True)

    refresh = goalRefresher()

    # Setup subsribers to the necessary topics
    rospy.Subscriber("/move_base/current_goal", PoseStamped, refresh.goalCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, refresh.posCallback)
    
    # Calls the refresh function at a rate of 1 hz
    rospy.Subscriber("/one_hz", Int16, refresh.refresher)

    rospy.spin()