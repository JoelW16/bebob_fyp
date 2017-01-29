#!/usr/bin/env python
from collections import deque
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import argparse

class image_converter:

    def __init__(self):

        self.ap = argparse.ArgumentParser()
        self.ap.add_argument("-b", "--buffer", type=int, default=64)
        args = vars(self.ap.parse_args())

        self.image_pub = rospy.Publisher("openCV_image", Image, queue_size=10)

        self.lower_blue = np.array([82, 216, 157])
        self.upper_blue = np.array([113, 255, 255])
        pts = deque(maxlen=args["buffer"])

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        def nothing(x):
            pass

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=5)


        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None

        if len(cnts) > 0:



            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center= (int(M["m10"] / M["m00"]),int( M["m01"] / M["m00"] ))

            if(center > (450,0)):
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Right', (520, 100), font, 4, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Left', (10, 100), font, 4, (255, 255, 255), 2, cv2.LINE_AA)

            if(radius > 10):
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0,255,255), 2)
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        else:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, 'No Ball', (10, 100), font, 4, (0, 0, 255), 2, cv2.LINE_AA)

        #res = cv2.bitwise_and(cv_image, cv_image, mask=mask)


        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1600, 1200)
        cv2.imshow('frame', cv_image)
        cv2.imshow('mask', mask)
        #cv2.imshow('Res', res)
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