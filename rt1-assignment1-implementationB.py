from __future__ import print_function

import time
from sr.robot import *

"""

"""

start_timer = time.time() 

a_th = 2.0
""" float: Threshold for the control of the orientation"""

d_grab = 0.4
""" float: Threshold for the control of the linear distance (grabbing the silver)"""

d_leave = 0.6
""" float: Threshold for the control of the linear distance (leaving the silver)"""

paired_golden= []
""" list to know which golden tokens have already been paired"""

paired_silver = []
""" list to know which silver tokens have already been paired"""


silver = True
""" boolean: variable for letting the robot know if it has to look for a silver or for a golden marker"""

R = Robot()
""" instance of the class Robot"""

def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token

    Returns:
	dist (float): distance of the closest silver token (-1 if no silver token is detected)
	rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
    code: unique value used to identify each token
    """
    dist=100
    for token in R.see():  # verify that the token is not already paired
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER and token.info.code not in paired_silver:
            dist=token.dist
            rot_y=token.rot_y
            code=token.info.code
    if dist==100:
        return -1, -1, -1
    else:
        return dist, rot_y, code

def find_golden_token():
    """
    Function to find the closest golden token

    Returns:
	dist (float): distance of the closest golden token (-1 if no golden token is detected)
	rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
    code: unique value used to identify each token
    """
    dist=100
    for token in R.see(): # verify that the token is not already paired
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD and token.info.code not in paired_golden:
            dist=token.dist
            rot_y=token.rot_y
            code=token.info.code
    if dist==100:
        return -1, -1, -1
    else:
        return dist, rot_y, code
    
def go_to_token(silver):
    return 1

    
    

while 1:
		
    if silver == True: # if silver is True, than we look for a silver token, otherwise for a golden one
        dist, rot_y, code = find_silver_token()   
    else:
        dist, rot_y, code = find_golden_token()
     
    if dist==-1: # if no token is detected, we make the robot turn 
        if len(paired_golden) == 6:
			print("All tokens collected")
			end_timer = time.time()
			print("Time: ", end_timer - start_timer)
			exit()
        print("I don't see any token!!")
        turn(+12, 1)
        
    elif silver and dist < d_grab: # if we are looking for a silver token and we are close to one, we try grab it.
        
        print("Found it!")
        
        if R.grab(): 
            print("Gotcha!")
            paired_silver.append(code) # if we grab the silver token, we add the code of the silver token to the list
            silver = not silver # we modify the value of the variable silver, so that in the next step we will look for the other type of token
            turn(20, 1) # we turn to avoid crossing the center of the map
        else:
            print("Aww, I'm not close enough.")
            
    elif not silver and dist < d_leave:  # if we are looking for a golden token and we are close to one, we release the silver token
        
        print("Found it!")
        R.release()
        print("There you go!")
        paired_golden.append(code) # when we release the silver token, we add the code of the golden pair to the list
        silver = not silver # we modify the value of the variable silver, so that in the next step we will look for the other type of token
        drive(-20,1)
        turn(20, 1)
                
    elif -a_th <= rot_y <= a_th: # if the robot is well aligned with the token, we go forward
        drive(40, 0.25)
    elif rot_y < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
        print("Left a bit...")
        turn(-4, 0.25)
    elif rot_y > a_th:
        print("Right a bit...")
        turn(+4, 0.25)
        

		

    
    

