#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ros_myo.msg import MyoPose
from std_msgs.msg import Empty


class myoTeleop():

    def __init__(self):
        self._pub_cmd = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

        rospy.Subscriber("/myo_raw/myo_gest", MyoPose, self.callback)
        self._hz = 30

        self._pan_rate = 0.3

        self._pose = 100
        self._pan = 0

        self.run()

    def run(self):
        rate = rospy.Rate(self._hz)
        while True:
            self._set_velocity()
            rate.sleep()

    movement_bindings = {
        1: 0, # idle
        3: -1, # pan left
        4: 1, # pan Right
    }

    def callback(self, data):
        self._myo_set_pose(data.pose)


    def _get_twist(self, pan):
        twist = Twist()
        twist.angular.z = pan
        return twist

    def _set_velocity(self):
        print self._pose

        pan = 0.0

        p = 0

        if self._pose in self.movement_bindings:
            p = self.movement_bindings[self._pose]

            pan += p

            pan = pan * self._pan_rate

            panLim = self._pan + pan

            if panLim > 35:
                self._pan = 35
            elif panLim < -35:
                self._pan = -35
            else:
                self._pan = panLim
            self._publish()
        else:
            if self._pose == 0:
                rospy.signal_shutdown('Bye')

    def _myo_set_pose(self, pose):
        print pose
        self._pose = pose


    def _publish(self):
        twist = self._get_twist(self._pan)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    rospy.init_node('myo_telop')
    myo_teleop = myoTeleop()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

