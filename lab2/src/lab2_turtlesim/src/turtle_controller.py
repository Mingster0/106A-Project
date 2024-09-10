#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
import sys

# Define the method which contains the node's main functionality
def controller(args):

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /user_messages
    pub = rospy.Publisher(f'{args[1]}/cmd_vel', Twist, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)

        user_input = input("Please enter a control (WASD) and press <Enter>: ")

        twist_msg = None
        if user_input == "w": 
            print('PRESSED w')
            twist_msg = Twist()
            twist_msg.linear.x = 1.0
            print(twist_msg)
        elif user_input == "a":
            print('PRESSED a')
            twist_msg = Twist()
            twist_msg.angular.z = -1.0
            print(twist_msg)
        elif user_input == "s":
            print('PRESSED s')
            twist_msg = Twist()
            twist_msg.linear.x = -1.0
            print(twist_msg)
        elif user_input == "d": 
            twist_msg = Twist()
            twist_msg.angular.z = 1.0
        else: 
            continue
        
        # Publish our string to the 'chatter_talk' topic
        pub.publish(twist_msg)
        # print(rospy.get_name() + ": I sent \"%s\"" % pub_string)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    args = sys.argv

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node(f'{args[1]}', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        controller(args)
    except rospy.ROSInterruptException: pass

