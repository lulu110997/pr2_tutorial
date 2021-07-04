#!/usr/bin/env python3
import pr2_controllers_msgs.msg as pr2_msgs # Need the .msg to import the messages. Don't import the package. Package has no attribute 'msg'
import actionlib
import rospy
import actionlib_tut2.msg as asd

def gripper():
	# Create a client that will send a request to the server
	gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", pr2_msgs.Pr2GripperCommandAction)
	gripper_client.wait_for_server()
	a = open_gripper(gripper_client)
	b = close_gripper(gripper_client)
	return a, b
def open_gripper(gripper_client):
	# Creates a goal to send to the action server.
	open_goal = pr2_msgs.Pr2GripperCommandGoal()
	open_goal.command.position = 0.08
	open_goal.command.max_effort = -1

	# Sends the goal to the action server using the client.
	gripper_client.send_goal(open_goal)
	gripper_client.wait_for_result()
	return gripper_client.get_result()

def close_gripper(gripper_client):
	close_goal = pr2_msgs.Pr2GripperCommandGoal()
	close_goal.command.position = 0.01
	close_goal.command.max_effort = 10

	gripper_client.send_goal(close_goal)
	gripper_client.wait_for_result()
	return gripper_client.get_result()

rospy.init_node('random') 
open_success, close_success = gripper()
print(f"opening details:\n{open_success}, closing details:\n{close_success}")
# help(asd)