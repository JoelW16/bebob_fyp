#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("openCV_image", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        def nothing(x):
            pass

        cv2.namedWindow('frame')
        cv2.createTrackbar('Hh', 'frame', 0, 255, nothing)
        cv2.createTrackbar('Hl', 'frame', 0, 255, nothing)
        cv2.createTrackbar('Sh', 'frame', 0, 255, nothing)
        cv2.createTrackbar('Sl', 'frame', 0, 255, nothing)
        cv2.createTrackbar('Vh', 'frame', 0, 255, nothing)
        cv2.createTrackbar('Vl', 'frame', 0, 255, nothing)

        self.Hh = cv2.getTrackbarPos('Hh', 'frame')
        self.Hl = cv2.getTrackbarPos('Hl', 'frame')
        self.Sh = cv2.getTrackbarPos('Sh', 'frame')
        self.Sl = cv2.getTrackbarPos('Sl', 'frame')
        self.Vh = cv2.getTrackbarPos('Vh', 'frame')
        self.Vl = cv2.getTrackbarPos('Vl', 'frame')

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([self.Hl, self.Sl, self.Vl])
        upper_blue = np.array([self.Hh, self.Sh, self.Vh])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        cv2.imshow('frame', cv_image)
        cv2.imshow('mask', mask)
        cv2.imshow('Res', res)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)






def main(args):
    ic = image_converter()
    rospy.init_node('image_converter')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)