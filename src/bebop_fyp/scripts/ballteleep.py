#!/usr/bin/env python
import rospy
import rospy
from std_msgs.msg import String, Int8


class BallTeleop():

    def __init__(self):
        rospy.init_node('camera_listener', anonymous=True)

    def run(self):
        rospy.Subscriber("ball_teleop", Int8, self.callback)
        rospy.spin()

    def lookup(data):
        return {
            1: 'Center',
            2: 'Top Center',
            3: 'Bottom Center',
            4: 'Left Center',
            5: 'Right Center',
            6: 'Right Bottom',
            7: 'Right Top',
            8: 'Left Bottom',
            9: 'Left Top',
        }[data]

    def callback(self):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.lookup(self.data))

if __name__ == '__main__':
    rospy.init_node('camera_listener', anonymous=True)
    ball_teleop = BallTeleop
    ball_teleop.run()