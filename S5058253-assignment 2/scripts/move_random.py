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

active_ = False # indicate request data
res_  = False # indicate receiving destination pos
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

# service callback
def move_r(req):
	global active_, desired_position_, res_
	active_ = req.data
	res = SetBoolResponse()
	res.success = False
	res.message = 'Done!'
	# if requested req.data=True get destination pos 
	# and send the goal to move_base/goal topic
	if active_: 
		res.success = True
		desired_position_.x = rospy.get_param('des_pos_x')
		desired_position_.y = rospy.get_param('des_pos_y')
		set_goal(desired_position_.x, desired_position_.y)	   
		print("Please wait ...")
		time.sleep(2)
		res_ = True 

	return res

def clbk_odom(msg):
	global position_
	position_ = msg.pose.pose.position

def main():
	global pub2 
	global position_, desired_position_
	global active_, res_

	rospy.init_node('move_random')
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	pub2 = rospy.Publisher('/move_base/goal',MoveBaseActionGoal, queue_size=20) #queue size????

	srv = rospy.Service('move_random', SetBool, move_r)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		if not (active_ and res_):
			res_ = False
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
