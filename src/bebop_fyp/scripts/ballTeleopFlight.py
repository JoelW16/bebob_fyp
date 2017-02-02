#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist


class BallTeleop():

    def __init__(self):
        rospy.init_node('camera_listener', anonymous=True)
        self._pub_cmd = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self._last_instruction = {}

        self._hz = 10

        # Set default values for Takeoff/Land
        self._takeoff = False
        self._land = False

        # Set rate for each axis retrives values from drone
        self._pitch_rate = rospy.get_param('~PilotingSettingsMaxTiltCurrent', 0.1)
        self._roll_rate = rospy.get_param('~PilotingSettingsMaxTiltCurrent', 0.1)
        self._yaw_rate = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 0.1)
        self._vertical_rate = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 0.1)

        # Set default values for axis
        self._pitch = 0
        self._roll = 0
        self._yaw = 0
        self._vertical = 0

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
        0: (0, 0, 0, 0),  # Hover
        0: (0, 0, 0, 0),  # Hover
        1: (0, 0, 0, 0), #Hover
        2: (0, 0, 0, 0),  # Hover
        3: (0, 0, 0, 0),  # Hover
        6: (0, 0, 0, 0),  # Hover
        7: (0, 0, 0, 0),  # Hover
        8: (0, 0, 0, 0),  # Hover
        9: (0, 0, 0, 0),  # Hover
        #: ( 1,  0,  0,  0), #Forward
        #: (-1,  0,  0,  0), #Backward
        #: ( 0,  1,  0,  0), #Left
        #: ( 0, -1,  0,  0), #Right
        4: (0, 0, 1, 0), #Rotate Counter Clockwise
        5: (0, 0, -1, 0), #Rotate Clockwise
        #2: ( 0,  0,  0,  1), #Ascend
        #3: ( 0,  0,  0, -1), #Descend
    }

    def callback(self, data):
        rate = rospy.Rate(self._hz)
        self._ball_zone(data.data)
        self._set_velocity()
        self._publish()

    def _get_twist(self, pitch, roll, yaw, vertical):
        twist = Twist()
        twist.linear.x = pitch
        twist.linear.y = roll
        twist.angular.z = yaw
        twist.linear.z = vertical
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        commands = []
        for a in self._last_instruction:
            if now - self._last_instruction[a] < 0.4:
                commands.append(a)
        pitch = 0.0
        roll = 0.0
        yaw = 0.0
        vertical = 0.0

        for c in commands:
            p, r, y, v = self.movement_bindings[c]
            pitch += p
            roll += r
            yaw += y
            vertical += v

        pitch *= self._pitch_rate
        roll *= self._roll_rate
        yaw *= self._yaw_rate
        vertical *= self._vertical_rate

        self._pitch = pitch
        self._roll = roll
        self._yaw = yaw
        self._vertical = vertical

    def _ball_zone(self, zone):
        if zone in self.movement_bindings:
            self._last_instruction[zone] = rospy.get_time()

    def _publish(self):
        rospy.loginfo(self._yaw)
        twist = self._get_twist(self._pitch, self._roll, self._yaw, self._vertical)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    ball_teleop = BallTeleop()
    ball_teleop.run()