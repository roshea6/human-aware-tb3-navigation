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
from visualization_msgs.msg import MarkerArray
import rosservice
import copy

# Callback function for the gazebo model states
def modelCallback(states_msg):
    # People publisher
    ppl_pub = rospy.Publisher('people', People, queue_size=1)

    # Marker publisher
    marker_pub = rospy.Publisher('people_viz_arr', MarkerArray, queue_size=1)

    # List to hold the indexes of the people objects in the message
    ppl_idxs = []

    # Loop through the names array to determine how many people objects are in the sim and their indexes
    for i in range(len(states_msg.name)):
        model_name = states_msg.name[i]

        # Check if the model name is that of person
        if model_name[:-2] == "person_standing":
            # Append the index to the list
            ppl_idxs.append(i)

    # # Marker array to hold the markers for each person
    # marker_list = MarkerArray()

    # # Create marker message and initialize with values for external person model
    # people_marker = Marker()
    # people_marker.header.frame_id = "map"
    # # people_marker.ns = "model"
    # people_marker.type = Marker.MESH_RESOURCE
    # people_marker.action = Marker.MODIFY
    # people_marker.mesh_resource = "package://social_nav_simulation/gazebo/models/human/meshes/standing.dae"
    # people_marker.mesh_use_embedded_materials = True
    # people_marker.scale.x = 1.0
    # people_marker.scale.y = 1.0
    # people_marker.scale.z = 1.0

    # Create people and person msgs
    all_people = People()
    one_person = Person()


    # Loop through all the indexes of the people models found
    for i in range(len(ppl_idxs)):
        # Get the index of the person model
        idx = ppl_idxs[i]

        # Add the person's model name
        one_person.name = states_msg.name[idx]

        # Add the person's x and y position
        one_person.position.x = states_msg.pose[idx].position.x
        one_person.position.y = states_msg.pose[idx].position.y

        # Set name of marker
        # people_marker.ns = one_person.name

        # # Create marker at the person's position
        # people_marker.pose.position.x = one_person.position.x
        # people_marker.pose.position.y = one_person.position.y
        # people_marker.pose.orientation = states_msg.pose[idx].orientation

        # marker_list.markers.append(copy.deepcopy(people_marker))

        # marker_pub.publish(people_marker)

        # Add the person's velocity
        one_person.velocity.x = 0#states_msg.twist[idx].linear.x
        one_person.velocity.y = 0 #states_msg.twist[idx].linear.y

        # ! Need to append a deep copy of the person object or or else each one will get overwritten
        # by the last person object appeneded to the list
        person_to_append = copy.deepcopy(one_person)

        # Add the person to the people message
        all_people.people.append(person_to_append)

    # Add paraent frame and time to header
    all_people.header.frame_id = "map"
    all_people.header.stamp = rospy.Time.now()

    # Publish list of all people
    ppl_pub.publish(all_people)

    # Publish list of people markers
    # marker_pub.publish(marker_list)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('gazebo_people_pub', anonymous=True)

    # Subscribe to the Gazebo model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, modelCallback)

    rospy.spin()