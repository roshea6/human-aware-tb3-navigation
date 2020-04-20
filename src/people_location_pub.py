#!/usr/bin/env python

"""
Description: Node to listen for gazebo model states messages, parse them for people models, 
and then publish their positions to the people topic

Authors: Ryan

"""

import rospy
from gazebo_msgs.msg import ModelStates
from people_msgs.msg import People
from people_msgs.msg import Person
from visualization_msgs.msg import Marker

# Callback function for the gazebo model states
def modelCallback(states_msg):
    # People publisher
    ppl_pub = rospy.Publisher('people', People, queue_size=1)

    # Marker publisher
    # marker_pub = rospy.Publisher('people_viz', Marker, queue_size=1)

    # List to hold the indexes of the people objects in the message
    ppl_idxs = []

    # Loop through the names array to determine how many people objects are in the sim and their indexes
    for i in range(len(states_msg.name)):
        model_name = states_msg.name[i]

        # Check if the model name is that of person
        if model_name[:-2] == "person_standing":
            # Append the index to the list
            ppl_idxs.append(i)

    # Create people and person msgs
    all_people = People()
    one_person = Person()

    # Clear the list of people
    # all_people.people.clear()

    # Add the person's model name
    one_person.name = states_msg.name[2]

    # Add the person's x and y position
    one_person.position.x = states_msg.pose[2].position.x
    one_person.position.y = states_msg.pose[2].position.y

    # Add the person's velocity
    # one_person.velocity.x = states_msg.twist[2].linear.x
    # one_person.velocity.y = states_msg.twist[2].linear.y

    all_people.header.frame_id = "map"
    all_people.header.stamp = rospy.Time.now()

    # Add the person to the people message
    all_people.people.append(one_person)

    # print states_msg.pose[2]

    ppl_pub.publish(all_people)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('gazebo_people_pub', anonymous=True)

    # Subscribe to the Gazebo model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, modelCallback)

    rospy.spin()