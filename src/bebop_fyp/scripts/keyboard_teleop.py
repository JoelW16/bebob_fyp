#!/usr/bin/env python

import curses
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=15):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class keyTelop():

    def __init__(self, interface):
        self._interface = interface

        #Define Publishers:
        #Takeoff/Land Empty std_msg
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        #Pitch, Roll, Yaw, Vertical geometry_msg/Twist
        self._pub_cmd = rospy.Publisher('bebop/cmd_vel', Twist)

        self._last_pressed = {}
        self._hz = 10

        #Set default values for Takeoff/Land
        self._takeoff = False
        self._land = False

        #Set rate for each axis retrives values from drone
        self._pitch_rate = rospy.get_param('~PilotingSettingsMaxTiltCurrent', 0.1)
        self._roll_rate = rospy.get_param('~PilotingSettingsMaxTiltCurrent', 0.1)
        self._yaw_rate = rospy.get_param('~SpeedSettingsMaxRotationSpeedCurrent', 0.1)
        self._vertical_rate = rospy.get_param('~SpeedSettingsMaxVerticalSpeedCurrent', 0.1)

        #Set default values for axis
        self._pitch = 0
        self._roll = 0
        self._yaw = 0
        self._vertical = 0

    #Define keys and map (pitch, roll, yaw, vertical)
    movement_bindings = {
        ord('w'): ( 1,  0,  0,  0), #Forward
        ord('s'): (-1,  0,  0,  0), #Backward
        ord('a'): ( 0,  1,  0,  0), #Left
        ord('d'): ( 0, -1,  0,  0), #Right
        ord('q'): ( 0,  0,  1,  0), #Rotate Counter Clockwise
        ord('e'): ( 0,  0, -1,  0), #Rotate Clockwise
        ord('r'): ( 0,  0,  0,  1), #Ascend
        ord('f'): ( 0,  0,  0, -1), #Descend
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    #Function sets the twist paramiters for linear and angular
    def _get_twist(self, pitch, roll, yaw, vertical):
        twist = Twist()
        twist.linear.x = pitch
        twist.linear.y = roll
        twist.angular.z = yaw
        twist.linear.z = vertical
        return twist


    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        pitch = 0.0
        roll = 0.0
        yaw = 0.0
        vertical = 0.0

        for k in keys:
            p, r, y, v = self.movement_bindings[k]
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

    def _key_pressed(self, keycode):
        if keycode == ord('x'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode == ord('o'):
            self._pitch = 0
            self._roll = 0
            self._yaw = 0
            self._vertical = 0
            self._land = False
            self._takeoff = False
        elif keycode == ord('t'):
            self._takeoff  = True
        elif keycode == ord('g'):
            self._land = True
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(1, 'Forwards: w  Backward: s')
        self._interface.write_line(2, 'Left: a      Right: d')
        self._interface.write_line(3, 'CCW: q       CW: e')
        self._interface.write_line(4, 'Ascend: r    Desend: f')
        self._interface.write_line(5, 'Takeoff: t    Land: g')
        self._interface.write_line(8, 'Pitch: %f, Roll: %f, Yaw: %f, Vertical: %f' % (self._pitch, self._roll, self._yaw, self._vertical))
        self._interface.write_line(9, 'Takeoff: %r, Land: %r' % (self._takeoff, self._land))
        self._interface.write_line(12,'o to reset control values, x to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._pitch, self._roll, self._yaw, self._vertical)
        self._pub_cmd.publish(twist)

        if self._takeoff:
            self.pub_takeoff.publish(Empty())
            self._takeoff = False

        if self._land:
            self.pub_land.publish(Empty())
            self._land = False

def main(stdscr):
    rospy.init_node('camera_control_teleop')
    teleop = keyTelop(TextWindow(stdscr))
    teleop.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
