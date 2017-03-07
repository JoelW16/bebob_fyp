#!/usr/bin/env python
import rospy
import sys
import requests
import atexit
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as b
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged as gps

class server:
    def __init__(self):
        self.battery = 0

        self.longitude = 500;
        self.latitude = 500;
        self.altitude = 0;
        self.batt_sub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", b, self.callback)
        self.gps_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", gps, self.sendGPS)
        self.start()

    def callback(self, data):
        print (data.percent)
        self.battery = data.percent
        requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58b82b7d2cc965257c433dea',
                     data={'battery': self.battery})

    def sendGPS(self, data):

        self.longitude = data.longitude
        self.latitude = data.latitude
        self.altitude = data.altitude

        requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58b82b7d2cc965257c433dea',
                     data={'longitude': self.longitude, 'latitude': self.latitude, 'Altitude': self.altitude})

    def start(self):
        print("Drone connect")
        requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58b82b7d2cc965257c433dea',
                     data={'battery' : self.battery, 'connected': "true"})

def exit():
    print("Drone disconnect")
    requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58b82b7d2cc965257c433dea',
                 data={'battery' : 0, 'connected': "false", 'longitude': 500, 'latitude': 500, 'Altitude': 0})

def main(args):
    atexit.register(exit)
    rospy.init_node('api_drone')
    api = server()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')



if __name__ == '__main__':
    main(sys.argv)
