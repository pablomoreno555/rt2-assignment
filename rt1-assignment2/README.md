Research Track I - Assignment 2  
Pablo Moreno Moreno - S5646698 
----------------------

This repository contains the ROS package developed for the second assignment: *my_assignment_2*.

To see its documentation go to the following URL: https://pablomoreno555.github.io/rt1-assignment2/


## Required applications/packages
- You need to have the package *assignment_2_2022* within your workspace.
- You need to have the *konsole* application installed in your machine, since it is used to display the different terminals.


## Building and running the code
Once the previous pre-requisites are met, you can build the package by executing the command `catkin_make` within the root directory of your workspace.

Then, to be able to run the simulation, we first need to make executable the Python scripts corresponding to the three nodes: *node_a.py*, *node_b.py* and *node_c.py*, located within the `scripts` folder. To do that, we need to go to the `scripts` folder and execute the following command:
```console
chmod +x *.py
```

Finally, in order to run the simulation, a launch file has been created. Its name is *my_assignment_2.launch*. So, you can run the whole simulation by executing the following command:
```console
roslaunch my_assignment_2 my_assignment_2.launch
```

The following windows will pop up:
- **Gazebo**, where we can see the simulation of the robot within the environment.
- **RViz**, where we can also visualize the robot.
- **node_a**: the terminal associated to the action client (*node_a*).
- **node_b**: the terminal associated to the service node (*node_b*).
- **node_c**: the terminal associated to *node_c*.


## Content of the Repository
The package *my_assignment_2* is organized as follows:
- The `scripts` folder, which contains the Python scripts corresponding to the three nodes: *node_a.py*, *node_b.py* and *node_c.py*.
- The `msg` folder, which contains the definition of the custom message *PosVel*.
- The `srv` folder, which contains the definition of the custom service *GoalsResults*.
- The `launch` folder, that contains the launch file (*my_assignment_2.launch*).
- The *CMakeLists.txt* file.
- The package manifest (*package.xml*).


## Description of the nodes
- **node_a**. It implements an action client for the action service *reaching_goal*, which is implemented within the package *assignment_2_2022*. Through a small User Interface, it allows the user to set a new target (x, y) or to cancel the current target. It also publishes on the topic */pos_vel* the robot position and velocity using the custom message *PosVel*, composed by the fields *x*, *y*, *vel_x* and *vel_y*, by relying on the values published on the topic */odom*.
- **node_b**. It implements the service *GoalsResults*, which, when called, prints and returns the number of goals reached and cancelled.
- **node_c**. It subscribes to the topics */pos_vel* and */reaching_goal/goal* and prints the current distance from the robot to the target and the robot's average speed. It prints the information at a rate established by the the parameter *rate_node_c*, which we can modify in the launch file.


## Pseudo-code of *node_a*
```console
FUNCTION "callback_odometry":
	
	Declare a message of the custom type "PosVel"
	Get the values of position (x and y) and velocity (linear in x and angular in z) from the received message (of type "Odom")
	Fill the four fields of the "PosVel" message with the corresponding received values
	Publish the message via the topic "/pos_vel"
	

MAIN FUNCTION:

	Initialize a rospy node
	Subscribe to the topic "/odom" (of type "Odometry") setting "callback_odometry" as the associated callback function
	Advertise the topic "/pos_vel"
	Initialize the parameters related to the number of goals reached and cancelled
	Create a simple action client of the action service '/reaching_goal', of type 'PlanningAction'
	Wait until the action server has started up and started listening for goals
	
	while this node is running:
		
		Prompt the user to enter a new target (x, y)
		Create a goal to send to the action server, with the values entered by the user, of type 'PlanningGoal'
		Send the goal to the action server

		while the goal has not been reached:
	
			If we receive a status message equal to 3:
				Report in the console that the goal has been reached
				Increment the parameter related to the number of goals reached
				Exit the loop

			Elif the user presses the 'c' key:
				Cancel the current goal
				Report in the console that the goal has been cancelled
				Increment the parameter related to the number of goals cancelled
				Exit the loop
```


