#!/usr/bin/env python3

"""
.. module:: node_c
	:platform: Unix
	:synopsis: Python module for the node_c
	
.. moduleauthor:: Pablo Moreno

This node prints the current distance from the robot to the target and the robot's average speed. 

It prints this information at the rate established by the the parameter **rate_node_c**, which can be modified in the launch file.

Subscribes to:
	**/pos_vel** \n
	**/reaching_goal/goal**
	
"""

import rospy
from my_assignment_2.msg import PosVel
from assignment_2_2022.msg import PlanningActionGoal
import sys
import math

x = y = vel_x = vel_z = 0 # Current position and velocity of the robot
x_goal = y_goal = 999.99 # Current target position. It will remain at that initialized value until the first target is sent
avg_vel_x = avg_vel_z = 0 # Average speeds of the robot (linear in x and angular in z)
acum_vel_x = acum_vel_z = 0 # Needed to compute the average speeds
n = 0 # Counter, also needed to compute the average speeds



def callback_pos_vel(my_pos_vel):
	"""
	Callback function to get the current position and velocity of the robot and update its average speed.

	Args:
		my_pos_vel(**PosVel**): The robot's position (*x*, *y*) and velocity (*vel_x*, *vel_z*)

	This function will be executed every time a message is received via the topic **/pos_vel**. It gets the robot's x and y 
	position, its linear x velocity and its angular z velocity, and updates the average linear and angular speeds.
	
	"""

	global x, y, vel_x, vel_z
	x = my_pos_vel.x
	y = my_pos_vel.y
	vel_x = my_pos_vel.vel_x
	vel_z = my_pos_vel.vel_z
	
	global avg_vel_x, avg_vel_z, acum_vel_x, acum_vel_z, n
	n += 1
	acum_vel_x += vel_x
	avg_vel_x = acum_vel_x / n
	acum_vel_z += vel_z
	avg_vel_z = acum_vel_z / n
	


def callback_goal(target):
	"""
	Callback function to get the current position of the target.

	Args:
		target(**PlanningActionGoal**): The current goal sent to the action server

	This function will be executed every time a message is received via the topic **/reaching_goal/goal**. It extracts the *x* and
	*y* position coordinates from the message received, and updates the corresponding internal variables of this node.
	
	"""

	global x_goal, y_goal
	x_goal = target.goal.target_pose.pose.position.x
	y_goal = target.goal.target_pose.pose.position.y


def main():
	"""
	The **main()** function continuously prints the distance to the target and the robot's average speed.
	
	First, it initializes the ROS node (by relying on the `rospy <http://wiki.ros.org/rospy/>`_ module), subscribes to the
	topics **/pos_vel** and **/reaching_goal/goal**, and gets the rate of execution of this node from the ROS parameter server.
	Then, at the specified rate, it prints the robot's average *x* and *z* speeds, and after a goal has been set, it also
	continuously computes its distance to the robot and prints it in the terminal.
	
	"""

	# Initialize a rospy node
	rospy.init_node('node_c')
	
	# Subscribe to the topics /pos_vel and /reaching_goal/goal
	rospy.Subscriber("/pos_vel", PosVel, callback_pos_vel)
	rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, callback_goal)
	
	# Get the rate of execution of this node from the ROS parameter server (frequency in Hz)
	rate_node_c = rospy.get_param('rate_node_c')
	rate = rospy.Rate(rate_node_c)
    
	while not rospy.is_shutdown():
        
        # We can't compute the distance to the goal until the first goal has been set
		if (x_goal == 999.99 and y_goal == 999.99):
			print("No goal set yet")
        
        # Compute the distance from the robot to the target and print it	
		else:
			dist = math.dist([x, y], [x_goal, y_goal])
			print("Distance to the target:", dist)
		
		# Print the average speeds of the robot	
		print("Average vel_x:", avg_vel_x, ", Average vel_z:", avg_vel_z, "\n")
        
        # So that the frequency in which the node prints corresponds to the one established in the corresponding parameter
		rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
		
