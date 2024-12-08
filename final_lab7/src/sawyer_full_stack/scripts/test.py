import rospy
import moveit_commander
import sys

rospy.init_node('test_moveit_connection')
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("right_arm")
print("MoveIt! Connection Successful")