#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

import forward_kinematics as fks

def callback(message):
    fks.baxter_forward_kinematics_from_joint_state(message)
    # print(transformation_matrix)

def extract_joint_angles_listener():

    sub = rospy.Subscriber('robot/joint_states', JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('extract_joint_angles_listener', anonymous=True)

    extract_joint_angles_listener()