#! /usr/bin/env python

# import ros stuff
import rospy
import time
import math 

# import ros message
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray

# import ros service
from std_srvs.srv import *

pos_check = False
pos_receive = False

pub = None
srv_client_wall_follower_ = None
srv_client_user_interface_ = None
srv_client_random_pos_ = None
srv_client_move_random_ = None
srv_client_move_target_ = None

desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

status_id = 0 # the move_base/goal status (1=active, 3=succeed)
state_desc_ = ['Options','Move randomly','Move to target', 'Wall following', 'Stop']

def change_state(state_):
	global state_desc_, option_ 
	global srv_client_wall_follower_, srv_client_user_interface_, srv_client_random_pos_
	global srv_client_move_random_, srv_client_move_target_
	global pos_receive, pos_check
    
	log = " --> state changed: %s" % state_desc_[state_]
	rospy.loginfo(log)
	if state_ == 0:
		rospy.set_param("option", 0)
		resp = srv_client_user_interface_()
		option_ = 0

	elif state_ == 1:
		resp = srv_client_wall_follower_(False)
		resp = srv_client_move_target_(False)
		resp = srv_client_random_pos_()
		pos_receive = resp.success
		if pos_receive:
			resp = srv_client_move_random_(True)

	elif state_ == 2:
		resp = srv_client_wall_follower_(False)
		resp = srv_client_move_random_(False)
		resp1 = srv_client_move_target_(True)
		pos_check = resp1.success

	elif state_ == 3:
		resp = srv_client_move_target_(False)
		resp = srv_client_move_random_(False)
		resp = srv_client_wall_follower_(True)

	elif state_ == 4:
		resp = srv_client_move_target_(False)
		resp = srv_client_wall_follower_(False)
		resp = srv_client_move_random_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)

def position_status(msg):
	global status_id
	s_list = msg.status_list
	n = len(s_list)
	if n > 0 :
		G_status = s_list[n-1]
		status_id = G_status.status
	else:
		status_id = 55
    
def main():
	time.sleep(2)
	global option_, status_id, desired_position_
	global srv_client_wall_follower_, srv_client_user_interface_, srv_client_random_pos_ 
	global srv_client_move_random_, srv_client_move_target_ 
	global pub, pos_check, pos_receive 
	pos_check = False
	pos_receive = False

	rospy.init_node('control')

	sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, position_status)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	rospy.wait_for_service('wall_follower_switch')
	try:
		srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

	rospy.wait_for_service('user_interface2')
	try:
		srv_client_user_interface_ = rospy.ServiceProxy('/user_interface2', Empty)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

	rospy.wait_for_service('random_pos')
	try:
		srv_client_random_pos_ = rospy.ServiceProxy('/random_pos', Trigger)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)   

	rospy.wait_for_service('move_random')
	try:
		srv_client_move_random_ = rospy.ServiceProxy('/move_random', SetBool)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)  
    
	rospy.wait_for_service('move_to_target')
	try:
		srv_client_move_target_ = rospy.ServiceProxy('/move_to_target', SetBool)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)      

	change_state(0)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
        
		if option_ == 0:
			option_ = rospy.get_param('option')
			if option_ > 4 or option_ < 1:
				print("Option is not available")
				change_state(0)

		elif option_ == 1:
			if not pos_receive:
				change_state(1)
			else:
				if status_id == 3:
					change_state(4)
					print("----------Target is REACHED!----------") 
					pos_receive = False
					change_state(0)

		elif option_ == 2:
			if not pos_check:
				change_state(2)
			else:
				if status_id == 3:
					change_state(4)
					print("----------Target is REACHED!----------") 
					pos_check = False
					change_state(0)

		elif option_ == 3:
			change_state(3)
			change_state(0)

		elif option_ == 4:
			change_state(4)
			change_state(0)

		rate.sleep()


if __name__ == "__main__":
    main()
