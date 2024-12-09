import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from svgpathtools import svg2paths

class Trajectory:
    def __init__(self, total_time):
        self.total_time = total_time

    def target_pose(self, time):
        pass

    def target_velocity(self, time):
        pass

    def display_trajectory(self, num_waypoints=67, show_animation=False, save_animation=False):
        trajectory_name = self.__class__.__name__
        times = np.linspace(0, self.total_time, num=num_waypoints)
        target_positions = np.vstack([self.target_pose(t)[:3] for t in times])
        target_velocities = np.vstack([self.target_velocity(t)[:3] for t in times])
       
        fig = plt.figure(figsize=plt.figaspect(0.5))
        colormap = plt.cm.brg(np.fmod(np.linspace(0, 1, num=num_waypoints), 1))

        # Position plot
        ax0 = fig.add_subplot(1, 2, 1, projection='3d')
        ax0.scatter(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], c=colormap, s=2)
        ax0.set_title(f"{trajectory_name} position")

        # Velocity plot
        ax1 = fig.add_subplot(1, 2, 2, projection='3d')
        ax1.scatter(target_velocities[:, 0], target_velocities[:, 1], target_velocities[:, 2], c=colormap, s=2)
        ax1.set_title(f"{trajectory_name} velocity")

        if show_animation or save_animation:
            def func(num, line):
                line[0]._offsets3d = target_positions[:num].T
                line[0]._facecolors = colormap[:num]
                line[1]._offsets3d = target_velocities[:num].T
                line[1]._facecolors = colormap[:num]
                return line

            line_ani = animation.FuncAnimation(fig, func, frames=num_waypoints, fargs=([ax0, ax1],), interval=max(1, int(1000 * self.total_time / (num_waypoints - 1))), blit=False)

        plt.show()
        if save_animation:
            line_ani.save(f'{trajectory_name}.gif', writer='pillow', fps=60)
            print(f"Saved animation to {trajectory_name}.gif")
class LinearTrajectory(Trajectory):
    def __init__(self, start_position, goal_position, total_time):

        Trajectory.__init__(self, total_time)
        self.start_position = start_position
        self.goal_position = goal_position
        self.distance = self.goal_position - self.start_position
        self.acceleration = (self.distance * 4.0) / (self.total_time ** 2) # keep constant magnitude acceleration
        self.v_max = (self.total_time / 2.0) * self.acceleration # maximum velocity magnitude
        self.desired_orientation = np.array([0, 1, 0, 0])

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        if time <= self.total_time / 2.0:
            # TODO: calculate the position of the end effector at time t, 
            # For the first half of the trajectory, maintain a constant acceleration
            pos = self.start_position + 0.5 * self.acceleration * time ** 2
        else:
            # TODO: Calculate the position of the end effector at time t, 
            # For the second half of the trajectory, maintain a constant acceleration
            # Hint: Calculate the remaining distance to the goal position. 
            pos_1 = self.start_position + 0.5 * self.acceleration * (self.total_time / 2.0) ** 2
            new_t = time - self.total_time / 2.0
            pos =  pos_1 + self.v_max * new_t - 0.5 * self.acceleration * (new_t ** 2)
        return np.hstack((pos, self.desired_orientation))

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame
        transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.nd)*sign
array`
            desired body-frame velocity of the end effector
        """
        if time <= self.total_time / 2.0:
            # TODO: calculate velocity using the acceleration and time
            # For the first half of the trajectory, we maintain a constant acceleration

            
            linear_vel = self.acceleration * time
        else:
            # TODO: start slowing the velocity down from the maximum one
            # For the second half of the trajectory, maintain a constant deceleration

            new_t = time - self.total_time / 2
            linear_vel = self.v_max - self.acceleration * new_t
        return np.hstack((linear_vel, np.zeros(3)))

class CircularTrajectory(Trajectory):
    def __init__(self, center_position, radius, total_time):
        Trajectory.__init__(self, total_time)
        self.center_position = center_position
        self.radius = radius
        self.angular_acceleration = (2 * np.pi * 4.0) / (self.total_time ** 2) # keep constant magnitude acceleration
        self.angular_v_max = (self.total_time / 2.0) * self.angular_acceleration # maximum velocity magnitude
        self.desired_orientation = np.array([0, 1, 0, 0])

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        if time <= self.total_time / 2.0:
            # TODO: calculate the ANGLE of the end effector at time t, 
            # For the first half of the trajectory, maintain a constant acceleration
            

            theta = 0.5*self.angular_acceleration* time **2
        else:
            # TODO: Calculate the ANGLE of the end effector at time t, 
            # For the second half of the trajectory, maintain a constant acceleration
            # Hint: Calculate the remaining angle to the goal position. 
            theta1 = 0.5*self.angular_acceleration* (self.total_time / 2.0) **2
            new_t = time - self.total_time / 2
            theta = theta1 + self.angular_v_max * new_t - 0.5 * self.angular_acceleration * (new_t ** 2)
        pos_d = np.ndarray.flatten(self.center_position + self.radius * np.array([np.cos(theta), np.sin(theta), 0]))
        return np.hstack((pos_d, self.desired_orientation))


    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame
        transformations.

        Parameters
        ----------128,
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        if time <= self.total_time / 2.0:
            # TODO: calculate ANGULAR position and velocity using the acceleration and time
            # For the first half of the trajectory, we maintain a constant acceleration


            theta = 0.5*self.angular_acceleration* time **2
            theta_dot = self.angular_acceleration * time
        else:
            # TODO: start slowing the ANGULAR velocity down from the maximum one
            # For the second half of the trajectory, maintain a constant deceleration
            
            
            theta1 = 0.5*self.angular_acceleration* (self.total_time / 2.0) **2
            new_t = time - self.total_time / 2
            theta = theta1 + self.angular_v_max * new_t - 0.5 * self.angular_acceleration * (new_t ** 2)
            theta_dot = self.angular_v_max - self.angular_acceleration * new_t
        vel_d = np.ndarray.flatten(self.radius * theta_dot * np.array([-np.sin(theta), np.cos(theta), 0]))
        return np.hstack((vel_d, np.zeros(3)))

class ImageTrajectory(Trajectory):
    """"Trajectory for a given SVG image.
    """
    def __init__(self, waypoints, total_time):
        Trajectory.__init__(self, total_time)
        super().__init__(total_time)
        self.waypoints = waypoints
        self.total_time = total_time
        self.segment_time = total_time / (len(waypoints) - 1)
        self.desired_orientation = np.array([0, 1, 0, 0])

    def target_pose(self, time):
        segment_index = int(time // self.segment_time)
        if segment_index >= len(self.waypoints) - 1:
            return np.hstack((self.waypoints[-1], [0, 1, 0, 0]))

        start = self.waypoints[segment_index]
        end = self.waypoints[segment_index + 1]
        local_time = time % self.segment_time
        alpha = local_time / self.segment_time
        pos = (1 - alpha) * start + alpha * end
        return np.hstack((pos, self.desired_orientation))

    def target_velocity(self, time):
        segment_index = int(time // self.segment_time)
        if segment_index >= len(self.waypoints) - 1:
            return np.zeros(6)

        start = self.waypoints[segment_index]
        end = self.waypoints[segment_index + 1]
        vel = (end - start) / self.segment_time

        # #setting max velocity threshhold
        # for v in [0, 1, 2]:
        #     sign = 1
        #     if vel[v] < 0:
        #         sign = -1
        #     vel[v] = min(abs(vel[v]), 0.03)*sign

        return np.hstack((vel, np.zeros(3)))

""" 
CODE FOR TESTING ImageTrajectory GENERATION:
def parse_svg_to_waypoints(svg_file, num_waypoints=150):
    paths, _ = svg2paths(svg_file)
    all_points = []

    for path in paths:
        for i in np.linspace(0, 1, num_waypoints // len(paths)):
            point = path.point(i)
            all_points.append([point.real, point.imag, 0])  # Add z=0 for 2D shapes

    return np.array(all_points)

def create_trajectory_from_svg(svg_file, total_time):
    waypoints = parse_svg_to_waypoints(svg_file)
    trajectory = ImageTrajectory(waypoints, total_time)
    return trajectory """
if __name__ == '__main__':
    """
    Run this file to visualize plots of your paths. Note: the provided function
    only visualizes the end effector position, not its orientation. Use the 
    animate function to visualize the full trajectory in a 3D plot.
    """

    # path = LinearTrajectory(np.array([0, 0, 0]), np.array([.1, .1, .1]), 10)
    path = CircularTrajectory(np.array([0.2, 0.4, 0.6]), .3, 10)
    path.display_trajectory()