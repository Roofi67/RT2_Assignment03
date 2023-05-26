#! /usr/bin/env python3

"""
.. module:: Modality1
 :platform: Unix
 :synopsis: Module for the first modality.

.. moduleauthor:: Abdul Rauf <ar223565@gmail.com>

Subscribes to:
 none
Publishes to:
 none
 
This node is needed to the first modality of the program. The node makes the robot autonomously reach a x,y position inserted by the user on the UI interface. It uses actionlib, which permits to us to make the code simplier and better. It is really important to say that we have three parameters that we get from the UI:
 - ``active`` which is the variable determinating the status of the modality, if it's on or not.
 - ``desired_position_x`` which communicates the desired x position.
 - ``desired_position_y`` which communicates the desired y position.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *

msg = """ 
This node makes the robot autonomously reach a x,y position inserted by the user on the UI interface.
It uses actionlib, which permits to us to make the code simplier and better.
"""

# Assigning to local variables the data travelling around ROS' nodes.

goal_msg = MoveBaseGoal()				# Action client.
achieved = False					# Variable for defining if a goal was achieved or not.
active_ = 0						# ROS poarameter to block/unlock the mode 
desired_position_x = 0					# X desired coordinate 
desired_position_y = 0					# Y desired coordinate 

# Defining ret_status() function, this function updates when status value changes
# and it's the core of the action, because it changes the behaviour of the robot
# the n variable 

def ret_status(status, result):
	"""
	Function to define the functionalities of the action, like advising the user of the goal received and other useful messages. We want the action to do the following dues:
	 - Advising the user Advising the user that the goal was cancelled.
	 - Advising the user that the goal was achieved.
	 - Advising the user that the goal was aborted because the timer expired.
	 - Advising the user that the goal was not accepted.
	
	Args:
	 none
	Returns:
	 none
	"""
	global client
	global achieved

	if status == 2:
		print("Goal received a cancel request.")
		return
	if status == 3:
		print("Goal achieved!")
		achieved = True
		return
	if status == 4:
		print("Timeout expired. Goal aborted.")
		return
	if status == 5:
		print("The goal was not accepted.")
		return
	if status == 6:
		print("The goal received a cancel request when he didn't finish the task.")
		return
	if status == 8:
		print("The goal received a cancel request before it started executing. ")
		return

# SetGoal() is used to set the goal data in the goal_msg which will be published.
# Sends a goal to the ActionServer, and also registers callbacks.
# If a previous goal is already active when this is called. We simply forget about 
# that goal and start tracking the new goal. No cancel requests are made.

def SetGoal(x, y):
	global goal_msg
	global client
	goal_msg.target_pose.pose.position.x = x
	goal_msg.target_pose.pose.position.y = y
	client.send_goal(goal_msg, ret_status)


# Defining main() function.

def main():
	"""
	Function to start all the features. The function if active is toggled or not it will do different tasks, it will wait when it is 
	0 and it will check the situation of the robot and the task, obviously it is all helped by the action.
	
	Args:
		none.
	"""
	global client
	global goal_msg
	global achieved
	global desired_position_x, desired_position_y
	global active_

	client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)		# Action client.

	rospy.init_node('mode1') # Init node

	# Running the Client to start the communication with the action.

	client.wait_for_server()
	goal_msg.target_pose.header.frame_id = 'map'			
	goal_msg.target_pose.header.stamp = rospy.Time.now()	
	goal_msg.target_pose.pose.orientation.w = 1

	# Boolprint used in order to know if the previous state was printable.

	boolprint = 0
	print(msg) 

	while (1):
		
		# Updating the variables (including active_).

		active_ = rospy.get_param('active')
		desired_position_x = rospy.get_param('des_pos_x')
		desired_position_y = rospy.get_param('des_pos_y')

		# If we are in Idle state but a goal was not achieved we need to cancel the goal.
		# If active_ is turned to 0 we can idle the process and wait until the
		# first mode is asked by the user. In any case, we want to cancel 
		# the goal asked by the user.

		if active_ == 0:
			
			# If the robot is idle forced by the user we do this:

			if boolprint == 0 and achieved == False:
				print("mode 1 is currently in idle state. \n")
				client.cancel_goal()
				boolprint = 1

			# If the robot has achieved the goal.

			if achieved == True:
				boolprint = 1
				achieved = False

		# If active_ is turned to 1 we procede with the task of the process.

		elif active_ == 1:

			# If the prevoius state was Idle then we can set a new goal
			if boolprint == 1:
				print("The robot is moving towards your desired target. ")
				SetGoal(desired_position_x, desired_position_y)	# Here we decide to set a new goal.
				boolprint = 0	# If this mode will be blocked, then we have to be put in idle.


if __name__ == '__main__':
    main()
