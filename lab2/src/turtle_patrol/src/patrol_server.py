#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from std_srvs.srv import Empty
from turtle_patrol.srv import Patrol  # Service type
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose

import sys


def patrol_callback(request):
    rospy.wait_for_service('clear')
    rospy.wait_for_service('/turtle1/teleport_absolute')
    clear_proxy = rospy.ServiceProxy('clear', Empty)
    teleport_proxy = rospy.ServiceProxy(
        f'/{args[1]}/teleport_absolute',
        TeleportAbsolute
    )
    vel = request.vel  # Linear velocity
    omega = request.omega  # Angular velocity
    x = request.x
    y = request.y
    theta = request.theta
    pub1 = rospy.Publisher(
        f'/{args[1]}/cmd_vel', Twist, queue_size=50)
    pub2 = rospy.Publisher(
        f"/{args[1]}/pose", Pose, queue_size=50
    )
    cmd1 = Twist()
    cmd1.linear.x = vel
    cmd1.angular.z = omega

    cmd2 = Pose()
    cmd2.x = x
    cmd2.y = y
    cmd2.theta = theta
    # Publish to cmd_vel at 5 Hz
    rate = rospy.Rate(5)
    # Teleport to initial pose
    teleport_proxy(9, 5, np.pi/2)
    # Clear historical path traces
    clear_proxy()
    while not rospy.is_shutdown():
        pub1.publish(cmd1)  # Publish to cmd_vel
        pub2.publish(cmd2)
        rate.sleep()  # Sleep until 
    return cmd  # This line will never be reached

def patrol_server(args):
    # Initialize the server node for turtle1
    rospy.init_node(f'{args[1]}_patrol_server')
    # Register service
    rospy.Service(
        f'/{args[1]}/patrol',  # Service name
        Patrol,  # Service type
        patrol_callback  # Service callback
    )
    rospy.loginfo('Running patrol server...')
    rospy.spin() # Spin the node until Ctrl-C


if __name__ == '__main__':

    args = sys.argv

    patrol_server(args)

