#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Empty

def firstflight():
	"""Simple program publishes takeoff message, sleeps then publishes landing message.
	"""


	pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
	pub_land = rospy.Publisher('/bebop/land', Empty, queue_size = 10)

	rospy.init_node('firstflight', anonymous=True)
	rate = rospy.Rate(0.2)	

	rate.sleep()

	pub_takeoff.publish(Empty())
	print "Takeoff"

	rate.sleep()

	pub_land.publish(Empty())
	print "landing"

	rospy.spin()

def main(args):
	try:
		rospy.init_node('bebop_guide')
		firstflight()
	except KeyboardInterrupt:
		rospy.signal_shutdown('Bye')

if __name__ == '__main__':
	main(sys.argv)

