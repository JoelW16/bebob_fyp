#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ros_myo.msg import MyoPose
from std_msgs.msg import Empty


class myoTeleop():

    def __init__(self):
        self._pub_cmd = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        rospy.Subscriber("/myo_raw/myo_gest", MyoPose, self.callback)
        self._hz = 7

        self._yaw_rate = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 0.5)

        self._FlightState = False
        self._pose = 100
        self._yaw = 0

        self.run()

    def run(self):
        rate = rospy.Rate(self._hz)
        while True:
            self._set_velocity()
            rate.sleep()

    movement_bindings = {
        1: (0), # idle
        3: (-1), # pan left
        4: (1), # pan Right
    }

    def callback(self, data):
        self._myo_set_pose(data.pose)


    def _get_twist(self,yaw):
        twist = Twist()
        twist.angular.z = yaw
        return twist

    def _set_velocity(self):
        print self._pose

        yaw = 0.0


        if self._pose in self.movement_bindings:

            y = self.movement_bindings[self._pose]
            yaw = y

        else:
            if self._pose == 2:
                if self._FlightState:
                    self.pub_land.publish(Empty())
                    self._FlightState = False
                else:
                    self.pub_takeoff.publish(Empty())
                    self._FlightState = True

            elif self._pose == 0:
                self.pub_land.publish(Empty())
                self._FlightState = False

        yaw *= self._yaw_rate
        self._yaw = yaw
        self._publish()



    def _myo_set_pose(self, pose):
        self._pose = pose


    def _publish(self):
        print self._yaw
        twist = self._get_twist(self._yaw)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    rospy.init_node('myo_telop_Flight')
    myo_teleop = myoTeleop()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

