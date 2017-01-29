#!/usr/bin/env python

import curses
import rospy
from geometry_msgs.msg import Twist

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
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


class cameraTelop():

    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('bebop/camera_control', Twist)

        self._hz = 30

        self._tilt_rate = 0.5
        self._pan_rate = 0.5

        self._last_pressed = {}
        self._pan = 0
        self._tilt = 0

    movement_bindings = {
        ord('w'):    ( 1,  0),
        ord('s'):  (-1,  0),
        ord('a'):  ( 0, -1),
        ord('d'): ( 0, 1),
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

    def _get_twist(self, tilt, pan):
        twist = Twist()
        twist.angular.z = tilt
        twist.angular.y = pan
        return twist


    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
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

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode == ord('r'):
            self._tilt = 0
            self._pan = 0
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Pan: %f, Tilt: %f' % (self._pan, self._tilt))
        self._interface.write_line(5, 'Use w a s d keys to move, r to reset camera, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._pan, self._tilt)
        self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('camera_control_teleop')
    teleop = cameraTelop(TextWindow(stdscr))
    teleop.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
