#!/usr/bin/env python
import rospy
import sys
import requests
import numpy as np
import atexit
from ros_myo.msg import MyoPose


class server:
    def __init__(self):
        self.pose = 0
        self.pose_sub = rospy.Subscriber("/myo_raw/myo_gest", MyoPose, self.callback)
        self.start();

    def callback(self, data):
        self.pose = np.int_(data.pose)
        requests.put('http://52.56.154.153:3000/api/myoUpdateStatus/58b829c6280587252ed0bec8', data={'pose': self.pose})

    def sendGPS(self, data):
        print ("lat" + str(data.latitude))
        print ("long" + str(data.longitude))
        print ("alt" + str(data.altitude))

    def start(self):
        print("Myo connect")
        requests.put('http://52.56.154.153:3000/api/myoUpdateStatus/58b829c6280587252ed0bec8',
                     data={'connected': "true"})


def main(args):
    atexit.register(exit)
    rospy.init_node('api_myo')
    api = server()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

def exit():
    print("Myo disconnect")
    requests.put('http://52.56.154.153:3000/api/myoUpdateStatus/58b829c6280587252ed0bec8',
                 data={'connected': "false", 'pose': 0})

if __name__ == '__main__':
    main(sys.argv)
