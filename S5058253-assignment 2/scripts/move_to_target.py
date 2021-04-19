#! /usr/bin/env python

# import ros stuff
import rospy
import time
import math

# import ros message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseActionGoal

# import ros service
from std_srvs.srv import *

from array import *

active_ = False # indicate request data
check_ = False # indicate if pos is checked and sent
desired_position_ = Point() 
position_ = Point() 

# send a position goal to move_base/goal topic
def set_goal(x,y):
	ac_goal = MoveBaseActionGoal()
	ac_goal.goal.target_pose.header.frame_id = "map"
	ac_goal.goal.target_pose.pose.orientation.w = 1.0
	ac_goal.goal.target_pose.pose.position.x = x
	ac_goal.goal.target_pose.pose.position.y = y
	pub2.publish(ac_goal)

# check whether the input position is predefined or not
def check(x,y):
	targets = [[-4,-3],[-4,2],[-4,7],[5,-7],[5,-3],[5,1]]
	for i in range(0, 6):
		if targets[i][0]==x and targets[i][1]==y:
			return True
	return False

# service callback
def move_target(req):
	global active_, check_, desired_position_
	active_ = req.data
	res = SetBoolResponse()
	res.success = False
	res.message = 'Done!'
	# if requested req.data=True ask for a target
	if active_:
		print("Insert a position, please")
		x = float(raw_input('x :'))
		y = float(raw_input('y :'))
		c_ = check(x,y)
		# if the target was reachable send it to move_base/goal topic
		if c_:
			rospy.set_param("des_pos_x", x)
			rospy.set_param("des_pos_y", y)
			set_goal(x, y)
			desired_position_.x = x
			desired_position_.y = y	    
			print("Please wait ...")
			time.sleep(2)
			res.success = True
			check_ = True
	return res

def clbk_odom(msg):
	global position_
	position_ = msg.pose.pose.position

def main():
	global pub2 
	global position_, desired_position_
	global active_, check_

	rospy.init_node('move_to_target')

	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	pub2 = rospy.Publisher('/move_base/goal',MoveBaseActionGoal, queue_size=20) #queue size????

	srv = rospy.Service('move_to_target', SetBool, move_target)
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if not (active_ and check_):
			check_ = False
			continue

		# if new goal is sent print robot info
		else:
			err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
			print("------")
			print("target position = [%d ,%d ]" % (desired_position_.x,desired_position_.y))
			print("robot position = [%6.3f,%6.3f ]" % (position_.x,position_.y))
			print("distance from target = %6.3f" % err_pos )
		

		rate.sleep()

if __name__ == '__main__':
    main()
