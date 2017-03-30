#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist


class BallTeleop():

    def __init__(self):
        rospy.init_node('camera_listener', anonymous=True)
        self._pub_cmd = rospy.Publisher('bebop/camera_control', Twist, queue_size=1)

        self._hz = 30

        self._tilt_rate = 0.4
        self._pan_rate = 0.4

        self._last_instruction = {}
        self._pan = 0
        self._tilt = 0

    def run(self):
        rospy.Subscriber("ball_teleop", Int8, self.callback)
        rospy.spin()

    def lookup(self, data):
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

    movement_bindings = {
        1: (0, 0),
        2: (1, 0),
        3: (-1, 0),
        4: (0, -1),
        5: (0, 1),
        6: (-1, 1),
        7: (1, 1),
        8: (-1, -1),
        9: (1, -1),
    }

    def callback(self, data):
        rate = rospy.Rate(self._hz)
        if data.data != 0:
            self._ball_zone(data.data)
        self._set_velocity()
        self._publish()
        rate.sleep()

    def _get_twist(self, tilt, pan):
        twist = Twist()
        twist.angular.z = tilt
        twist.angular.y = pan
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_instruction:
            if now - self._last_instruction[a] < 0.4:
                keys.append(a)
        tilt = 0.0
        pan = 0.0
        for k in keys:
            t, p = self.movement_bindings[k]
            tilt += t
            pan += p

        tilt = tilt * self._tilt_rate
        pan = pan * self._pan_rate

        tiltLim = self._tilt + tilt
        panLim = self._pan + pan

        if tiltLim > 18:
            self._tilt = 18
        elif tiltLim < -40:
            self._tilt = -40
        else:
            self._tilt =tiltLim

        if panLim > 35:
            self._pan = 35
        elif panLim < -35:
            self._pan = -35
        else:
            self._pan = panLim

    def _ball_zone(self, zone):
        if zone in self.movement_bindings:
            self._last_instruction[zone] = rospy.get_time()

    def _publish(self):
        twist = self._get_twist(self._pan, self._tilt)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    ball_teleop = BallTeleop()
    ball_teleop.run()