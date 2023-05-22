Research Track I - Assignment 1  
Pablo Moreno Moreno - S5646698 
----------------------


This repository contains the Python script developed for the assignment, together with the Python Robotics Simulator.

The Python script that I have developed is located in the directory `\robot-sim\assignment.py`

If you are located in the `\robot-sim` directory, you can run it with the following command:

```bash
$ python2 run.py assignment.py
```

Below, you can find a pseudocode of the program:

```
Indicate that we start looking for a silver token

While we are inside the robot control loop:

	If we need to move to a silver token:
		Look for the closest silver token that has not been already paired

		If we don't see any unpaired silver token: 
			We rotate slowly until we find one
		
		Elif the robot is not well aligned with the token:
			We rotate it in the proper direction
		
		Elif we are close to the token:
			We try to grab it
			
			If we successfully grab it:
				Get the code corresponding to this silver token
				Update the list of silver tokens that have already been paired
				Indicate that in the next step we will look for a golden token
		
		Else:
			The robot is well aligned with the token, so we go forward
	

	Else:
		Look for the closest golden token that has not been already paired
		
		If we don't see any unpaired golden token:
			We rotate slowly until we find one

		Elif the robot is not well aligned with the token:
			We rotate it in the proper direction
		
		Elif we are close to the golden token:
			We try to release the silver token we are grabbing

			If we successfully release it:
				Get the code corresponding to the golden token we have moved to
				Update the list of golden tokens that have already been paired
				Indicate that in the next step we will look for a silver token
			
				If we have already paired the six golden tokens:
	 				Exit the program

		Else:
			The robot is well aligned with the token, so we go forward
		
```

As a possible improvement, we could implement a more intelligent way for the robot to choose how to make the pairs beteween the tokens. Right now, the robot just matches the silver token that it grabs with the first golden token that it detects, regardless of how far away it is from the current position of the robot. In order to save time and energy, it may be a good idea for the robot to analize first all the unpaired golden tokens, rotating 360º around itself, and then choose the closest one.
