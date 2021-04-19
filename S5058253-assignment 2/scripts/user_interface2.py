#! /usr/bin/env python

# import ros stuff
import rospy

# import ros service
from std_srvs.srv import *

# service callback

def set_new_pos(req):

	# ask user to choose on of the option
    print("Choose an options please")
    print("1)Move randomly  2)Move to target  3)Follow wall  4)Stop")
    op = int(raw_input('option n. :')) 

	# set 'option' parameter
    rospy.set_param("option", op)
    return []
    
def main():
    rospy.init_node('user_interface2')
    srv = rospy.Service('user_interface2', Empty, set_new_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
