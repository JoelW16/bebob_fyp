#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ros_myo.msg import MyoPose
from std_msgs.msg import Empty


class myoTeleop():

    def __init__(self):
        rospy.init_node('ball_teleop', anonymous=True)
        self._pub_cmd = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self._yaw_rate = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 0.5)

        self._yaw = 0

        self._FlightState = False
        self._hz = 100

        self._pose = 0
        self._last_instruction = {}


    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            rospy.Subscriber("/myo_raw/myo_gest", MyoPose, self.callback)
            self._set_velocity()
            self._publish()
            rate.sleep()


    movement_bindings = {
        0: (0), # unknown
        1: (0), # idle
        3: (-1), # yaw left
        4: (1), # yaw Right
    }


    def callback(self, data):
        rate = rospy.Rate(self._hz)
        self._myo_pose(data.pose)
        self._set_velocity()
        self._publish()
        rate.sleep()

    def _get_twist(self,yaw):
        twist = Twist()
        twist.angular.z = yaw
        return twist

    def _set_velocity(self):

        yaw = 0.0

        if self._pose in self.movement_bindings:
            y = self.movement_bindings[self._pose]
            yaw += y

        yaw *= self._yaw_rate
        self._yaw = yaw

    def _myo_pose(self, pose):
        print pose
        if pose in self.movement_bindings:
            self._last_instruction[pose] = rospy.get_time()
        else:
            if pose == 2:
                if self._FlightState:
                    self.pub_land.publish(Empty())
                    self._FlightState = False
                else:
                    self.pub_takeoff.publish(Empty())
                    self._FlightState = True




    def _publish(self):
        twist = self._get_twist(self._yaw)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    ball_teleop = myoTeleop()
    ball_teleop.run()
