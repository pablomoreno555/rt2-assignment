#!/usr/bin/env python3

"""
.. module:: node_b
	:platform: Unix
	:synopsis: Python module for the node_b
	
.. moduleauthor:: Pablo Moreno

This node implements the service **/goals_results**, which, when called, prints and returns the number of goals reached and 
cancelled.

Server of the service:
	**/goals_results**
	
"""

import rospy
import sys
from my_assignment_2.srv import GoalsResults, GoalsResultsResponse
# We need to import both, the whole service type and only the response part


def serverCallback(req):
	"""
	Callback function to implement the service **/goals_results**.

	Args:
		None: *req* is empty
	Return:
		GoalsResultsResponse(reached, cancelled): Number of goals reached and cancelled

	This function will be executed every time a **/goals_results** service is requested. It simply gets the value of the
	the parameters **goals_reached** and **goals_cancelled** from the ROS parameter server, prints them into the terminal and 
	returns them.
	
	"""

	reached = rospy.get_param('goals_reached')
	cancelled = rospy.get_param('goals_cancelled')
	print("\nNumber of goals reached:", reached, "\nNumber of goals cancelled:", cancelled)
	return GoalsResultsResponse(reached, cancelled)
	
	
def main():
	"""
	The **main()** function advertises the service **/goals_results**.
	
	It first initializes the ROS node (by relying on the `rospy <http://wiki.ros.org/rospy/>`_ module), then advertises
	the service, reports in the terminal that the service is ready, and spins forever, continuously checking
	if a service is requested.
	
	"""

	# Initialize a rospy node
	rospy.init_node('goals_results_server')
	
	# Advertise the service 'goals_results', of type 'GoalsResults'
	s = rospy.Service('goals_results', GoalsResults, serverCallback)
	
	print("Service ready! When called, it will provide the number of goals reached and cancelled.")
	
	# Wait forever, continuously checking if a service is requested and, if so, executing the callback function
	rospy.spin() 


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
		
