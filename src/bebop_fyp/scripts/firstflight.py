#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import Empty
		

def firstflight():
	pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
	pub_land = rospy.Publisher('/bebop/land', Empty, queue_size = 10)
	pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)

	
	command = Twist()
	command.linear.x  = 0 	#pitch
	command.linear.y  = 0	#roll
	command.linear.z  = 0	#z_vel
	command.angular.z = 0	#yaw

	

	rospy.init_node('firstflight', anonymous=True)
	rate = rospy.Rate(0.2)	

	rate.sleep()

	pub_takeoff.publish(Empty())
	print "Takeoff"

	rate.sleep()


	pub_land.publish(Empty())
	print "landing"



if __name__ == '__main__':
	try:
		firstflight()
	except rospy.ROSInterruptException:
		pass
