# Import the required libraries to use the robotics simulator and to work with time
from __future__ import print_function
import time
from sr.robot import *

# Threshold for the control of the linear distance
a_th = 2.0

# Threshold for the control of the orientation
d_th = 0.4

# Variable for letting the robot know if it has to look for a silver or for a golden token
silver = True

# Lists that store the silver and golden tokens that have already been paired
lst_silver_tokens_paired = []
lst_golden_tokens_paired = []

# Instance of the class Robot
R = Robot()

"""
Function for setting a linear velocity
Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
"""
def drive(speed, seconds):
	R.motors[0].m0.power = speed
	R.motors[0].m1.power = speed
	time.sleep(seconds)
	R.motors[0].m0.power = 0
	R.motors[0].m1.power = 0

"""
Function for setting an angular velocity
Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
"""
def turn(speed, seconds):
	R.motors[0].m0.power = speed
	R.motors[0].m1.power = -speed
	time.sleep(seconds)
	R.motors[0].m0.power = 0
	R.motors[0].m1.power = 0

"""
Function to find the closest silver token that has not been already paired
Args: the list of silver tokens (codes) that have already been paired
Returns: dist (float): distance to the silver token (-1 if no silver token is detected)
		 rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
		 code: identifier of the silver token (-1 if no silver token is detected)
"""
def find_silver_token(lst_silver_tokens_paired):
	
	# Maximum distance that the robot can see
	dist = 100
	
	# Iterate through all the detected tokens
	for token in R.see():
	
		# Check if this is the closest silver token detected so far
		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER:
		
			# Check that this token has not been already paired
			for silver_token_paired in lst_silver_tokens_paired:
				if token.info.code == silver_token_paired:
					break
					
			# Update the distance, angle, and code values to those corresponding to this token
			else:
				dist = token.dist
				rot_y = token.rot_y
				code = token.info.code
	
	# If did not find any unpaired silver token, return -1 in all the fields
	if dist == 100:
		return -1, -1, -1
		
	# Otherwise, return the corresponding values
	else:
		return dist, rot_y, code

"""
Function to find the closest golden token that has not been already paired
Args: the list of golden tokens (codes) that have already been paired
Returns: dist (float): distance to the golden token (-1 if no golden token is detected)
		 rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
		 code: identifier of the golden token (-1 if no golden token is detected)
"""
def find_golden_token(lst_golden_tokens_paired):

	# Maximum distance that the robot can see
	dist = 100
	
	# Iterate through all the detected tokens
	for token in R.see():
	
		# Check if this is the closest golden token detected so far
		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
		
			# Check that this token has not been already paired
			for golden_token_paired in lst_golden_tokens_paired:
				if token.info.code == golden_token_paired:
					break	
					
			# Update the distance, angle, and code values to those corresponding to this token
			else:
				dist = token.dist
				rot_y = token.rot_y
				code = token.info.code
			
	# If did not find any unpaired golden token, return -1 in all the fields
	if dist == 100:
		return -1, -1, -1
		
	# Otherwise, return the corresponding values
	else:
		return dist, rot_y, code


'''		
Robot Control Loop
'''
while 1:

	# In case we need to move to a silver token
	if silver:
		
		# Look for the closest silver token that has not been already paired
		dist_token, ang_token, code_token = find_silver_token(lst_silver_tokens_paired)

		# If we don't see any unpaired silver token, we rotate slowly until we find one
		if dist_token == -1:
			print("I don't see any unpaired silver token. Rotating...")
			turn(10, 1)
		
		# If the robot is not well aligned with the token, we rotate it in the proper direction
		elif ang_token < -a_th:
			print("Left a bit...")
			turn(-2, 0.5)
		elif ang_token > a_th:
			print("Right a bit...")
			turn(+2, 0.5)
		
		# If we are close to the token, we try to grab it
		elif dist_token < d_th: 
			print("Reached it!")
			if R.grab():
				print("Silver token successfully grabbed!")
				
				# Get the code corresponding to this silver token
				dist_token, ang_token, code_token = find_silver_token(lst_silver_tokens_paired)
				
				# Update the list of silver tokens that have already been paired
				lst_silver_tokens_paired.append(code_token)
				print("Silver tokens paired:", lst_silver_tokens_paired)
				
				# Modify the value of "silver", so that in the next step we look for a golden token
				silver = not(silver)

		# If the robot is well aligned with the token, we go forward
		else:
			print("Forward...")
			drive(30, 0.3)
	
	
	# In case we need to move to a golden token
	else:
	
		# Look for the closest golden token that has not been already paired
		dist_token, ang_token, code_token = find_golden_token(lst_golden_tokens_paired)

		# If we don't see any unpaired golden token, we rotate slowly until we find one
		if dist_token == -1:
			print("I don't see any unpaired golden token. Rotating...")
			turn(10, 1)
		
		# If the robot is not well aligned with the token, we rotate it in the proper direction
		elif ang_token < -a_th:
			print("Left a bit...")
			turn(-2, 0.5)
		elif ang_token > a_th:
			print("Right a bit...")
			turn(+2, 0.5)
		
		# If we are close to the golden token, we try to release the silver token we are grabbing
		elif dist_token < 1.4*d_th: 
			print("Reached it!")
			if R.release():
				print("Silver token successfully released!")
				
				# Get the code corresponding to the golden token we have moved to
				dist_token, ang_token, code_token = find_golden_token(lst_golden_tokens_paired)
				
				# Update the list of golden tokens that have already been paired
				lst_golden_tokens_paired.append(code_token)
				print("Golden tokens paired:", lst_golden_tokens_paired)
				
				# Modify the value of "silver", so that in the next step we look for a silver token
				silver = not(silver)
				
				# If we have already paired the six golden tokens located in the arena, exit the program
				if len(lst_golden_tokens_paired) == 6:
					print("Done!")
					exit()

		# If the robot is well aligned with the token, we go forward
		else:
			print("Forward...")
			drive(30, 0.3)
		    
