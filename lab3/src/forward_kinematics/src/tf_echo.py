#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import tf2_ros as tf2

import sys

import time


def tf_echoer(args):



    target_frame = args[1]
    source_frame = args[2]

    tfBuffer = tf2.Buffer()
    tf2.TransformListener(tfBuffer)

    t_end = time.time() + 10

    while time.time() < t_end:

        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            print(trans)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            print(e)


if __name__ == '__main__':

    args = sys.argv

    rospy.init_node('tf2_echoer')

    tf_echoer(args)