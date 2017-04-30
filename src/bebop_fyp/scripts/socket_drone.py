#!/usr/bin/env python
import rospy
import sys
import json
from guide_flight import guide
from socketIO_client import SocketIO, BaseNamespace
from std_msgs.msg import Empty

class socket:

    def __init__(self):
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.pub_emgland = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
        self.startSocket()

    def startSocket(self):
        self.sio = SocketIO('fyp.joelwalker.co.uk', 3000, BaseNamespace)
        self.sio.on('connected', self.on_connected)
        self.sio.on('updateDronePos', self.on_updatePos)
        self.sio.on('takeoff', self.on_takeoff)
        self.sio.on('landing', self.on_landing)
        self.sio.on('emgLanding', self.on_emgLanding)
        self.sio.emit('connected')
        self.sio.wait()

    def on_connected(self, *args):
        print ("Socket Connected")

    def on_updatePos(self, *args):
        dataj = json.loads(json.dumps(args))[0]

        latitude = dataj['latitude']
        longitude = dataj['longitude']
        print("New Node")
        print("lat: " + str(latitude))
        print("long: " + str(longitude))

        node = guide(latitude, longitude)


    def on_takeoff(self, args):
        print("takeoff")
        self.pub_takeoff.publish(Empty())

    def on_landing(self, args):
        print("land")
        self.pub_land.publish(Empty())

    def on_emgLanding(self, args):
        print("EMG LANDING")
        self.pub_emgland.publish(Empty())






def main(args):
    try:
        rospy.init_node('bebop_socket')
        rospy.spin()
    except KeyboardInterrupt:

        rospy.signal_shutdown('Bye')



if __name__ == '__main__':
    main(sys.argv)
