#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch
import image_preprocessing

from paths.trajectories import LinearTrajectory, CircularTrajectory, ImageTrajectory
from paths.paths import MotionPath
from paths.image_path import ImagePath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Terminal Notes
# python main.py -task line -ar_marker 3 2 8 -img nike_logo.svg --log


def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

# def publish_board_marker(tag_positions, marker_pub):

#     """
#     Publish a marker to visualize the drawing board in RViz.

#     Parameters
#     ----------
#     tag_positions : dict
#         Positions of the AR tags (bottom_left, top_left, bottom_right).

#     marker_pub : rospy.Publisher
#         Publisher for RViz markers.

#     """

#     bottom_left = tag_positions['bottom_left']
#     top_left = tag_positions['top_left']
#     bottom_right = tag_positions['bottom_right']
    
#     # Calculate the fourth corner (top_right)
#     top_right = bottom_right + (top_left - bottom_left)
    
#     # Define the corners of the board
#     corners = [bottom_left, bottom_right, top_right, top_left]
    
#     # Create a Marker for the board
#     marker = Marker()
#     marker.header.frame_id = "base"
#     marker.header.stamp = rospy.Time.now()

#     marker.ns = "drawing_board"
#     marker.id = 0
#     marker.type = Marker.LINE_STRIP
#     marker.action = Marker.ADD
#     marker.scale.x = 0.01  # Line width
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0

#     # Add the corners to the marker
#     for corner in corners:
#         p = Point()
#         p.x, p.y, p.z = corner
#         marker.points.append(p)

#     # Close the rectangle by connecting the last point to the first
#     p = Point()
#     p.x, p.y, p.z = corners[0]
#     marker.points.append(p)

#     # Publish the marker
#     marker_pub.publish(marker)

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    
    # TODO: initialize a tf buffer and listener as in lab 3

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        # TODO: lookup the transform and save it in trans
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)

def get_trajectory(limb, kin, ik_solver, tag_pos, args):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = args.num_way
    task = args.task

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    # AR Tag Positions

    tag_positions = {
        'bottom_left': tag_pos[0],
        'top_left': tag_pos[1],
        'bottom_right': tag_pos[2],
    }

    # Calculate drawing plane properties

    bottom_left = tag_positions['bottom_left']
    bottom_left[2] = 0
    top_left = tag_positions['top_left']
    top_left[2] = 0
    bottom_right = tag_positions['bottom_right']
    bottom_right[2] = 0

    # Define the transformation matrix from drawing plane to robot base frame
    print(f'bottom_left: ', bottom_left)

    plane_origin = bottom_left

    # Moving 4 cm inwards to clear AR tag
    ar_tag_size = 0.08
    plane_origin[0] += ar_tag_size/2
    plane_origin[1] -= ar_tag_size /2
    #indexing has to be switched to be rel to base frame axes
    w_bound = abs(bottom_right[1] - bottom_left[1]) - ar_tag_size /2
    h_bound = abs(top_left[0] - bottom_left[0]) - ar_tag_size /2

    print("AR Tag Bounds: ", "Width ", w_bound, "Height: ", h_bound)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    if task == 'line':
        target_pos = tag_pos[0]
        target_pos[2] += 0.4 #linear path moves to a Z position above AR Tag.
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
    elif task == 'circle':
        target_pos = tag_pos[0]
        target_pos[2] += 0.5
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.1, total_time=20)
    elif task == 'image':
        target_pos = bottom_left
        total_time = 60
        print("DETECTED BOTTOM LEFT:", target_pos)


        processed_svg = image_preprocessing.main(args.img)

        img = ImagePath(processed_svg)
        
        waypoints = img.parse_svg_to_waypoints(num_way)
        plane_origin[2] += + 0.05 # moves to a Z position 1 cm above the AR tag.
        breakpoint()
        #TODO FIX SCALE AND CENTER FUNCTION
        scaled_waypoints = img.scale_and_center_waypoints(waypoints, plane_origin, h_bound, w_bound) 

        trajectory = ImageTrajectory(scaled_waypoints, total_time)
        trajectory.display_trajectory(num_way)
    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller


def main():
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='moveit', 
        help='Options: moveit, open_loop, pid.  Default: moveit'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('-img', type=str, default=None, help="path to SVG file.")

    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()

    #TODO VERIFY ADD ARGUMENT IS CORRECT


    rospy.init_node('moveit_node')
    
    tuck()
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # verify that image path exists'
    #TODO: implement correct one that passes this if there is no arg sent to img_path
    img_path = args.img
    if img_path is None:
        print("path argument check")
        # sys.exit()

    # Lookup the AR tag position.
    #TODO: Lookup for 3 markers, specifying which one is at a corner
   

    tag_pos = [lookup_tag(marker) for marker in args.ar_marker]

    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, args)

    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')
    
    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)

    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    if args.controller_name == "moveit":
        try:
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        planner.execute_plan(robot_trajectory)
    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        if not done:
            print('Failed to move to position')
            sys.exit(0)

if __name__ == "__main__":
    main()