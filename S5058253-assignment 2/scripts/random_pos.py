#! /usr/bin/env python

# import ros stuff
import rospy
import math
import random

# import ros service
from std_srvs.srv import *
from array import *


def set_rand_pos(req):

	# define an array of 6 specific positions
    targets = [[-4.0,-3.0],[-4.0,2.0],[-4.0,7.0],[5.0,-7.0],[5.0,-3.0],[5.0,1.0]]
	# generate a random integer from [0,5]
    n = random.randint(0, 5)
	# choose the random position
    x = targets[n][0]
    y = targets[n][1]
	# set destination parameter
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    res = TriggerResponse()
    res.success = True
    res.message = 'pos received!'
    return res

def main():
    rospy.init_node('random_pos')
    srv = rospy.Service('random_pos', Trigger, set_rand_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
