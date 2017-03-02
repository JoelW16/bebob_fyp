#!/usr/bin/env python
import rospy
import sys
import requests
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as b
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged as gps

class server:
    def __init__(self):
        self.battery = 0
        self.batt_sub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", b, self.callback)
        self.gps_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", gps, self.sendGPS)

    def callback(self, data):
        print (data.percent)
        self.battery = data.percent
        requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58ab72d5ff63d537a10e8a2c',
                     data={'battery': self.battery})

    def sendGPS(self, data):
        print ("lat" + str(data.latitude))
        print ("long" + str(data.longitude))
        print ("alt" + str(data.altitude))

    def shutdown(self):
        requests.put('http://52.56.154.153:3000/api/droneUpdateStatus/58ab72d5ff63d537a10e8a2c',
                     data={'battery': 0})

def main(args):
    rospy.init_node('api')
    api = server()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        api.shutdown()
        rospy.signal_shutdown('Bye')

if __name__ == '__main__':
    main(sys.argv)
