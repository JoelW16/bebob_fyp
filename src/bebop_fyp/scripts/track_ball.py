#!/usr/bin/env python
from collections import deque
import sys
import rospy
import cv2
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import argparse

class image_converter:
        #cv2.line(cv_image, (0, height/3), (width, height/3), (0, 255, 0), 1)
        #cv2.line(cv_image, (0, height -height/3), (width,height- height/3), (0, 255, 0), 1)
        #cv2.line(cv_image, (width/3, 0), (width/3, height), (255, 255, 255), 1)
        #cv2.line(cv_image, (width - width/3, 0), (width -width/3, height), (255, 255, 255), 1)
    def __init__(self):

        self.first = True
        self.ap = argparse.ArgumentParser()
        self.ap.add_argument("-b", "--buffer", type=int, default=64)
        args = vars(self.ap.parse_args())

        self.image_pub = rospy.Publisher("openCV_image", Image, queue_size=10)
        self.pub = rospy.Publisher('ball_teleop', Int8, queue_size=10)

        self.lower_blue = np.array([104, 172, 68])
        self.upper_blue = np.array([119, 255, 255])
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

        height = np.size(cv_image, 0)
        width = np.size(cv_image, 1)
        center_center_boundary = [(width/3, height/3), ((width - width/3), (height - height/3))]
        left_center_boundary = [(0, height/3), (width/3, height - height/3)]
        right_center_boundary = [(width/3, height/3), (width, height - height/3)]
        top_center_boundary = [(width/3, 0), (width-(width/3), height/3)]
        bottom_center_boundary = [(width/3, height - (height/3)), (width - (width / 3), height)]


        if self.first:
            print height, width
            print center_center_boundary
            self.first = False


        #cv2.line(cv_image, (width/2, 0), (width/2, height), (255, 255, 255), 1)
        #cv2.line(cv_image, (0, height/2), (width, height/2), (255, 255, 255), 1)


        cv2.rectangle(cv_image, left_center_boundary[0], left_center_boundary[1],  (255, 0, 0), 1)
        cv2.rectangle(cv_image, right_center_boundary[0], right_center_boundary[1], (255, 0, 0), 1)
        cv2.rectangle(cv_image, top_center_boundary[0], top_center_boundary[1], (255, 0, 0), 1)
        cv2.rectangle(cv_image, bottom_center_boundary[0], bottom_center_boundary[1], (255, 0, 0), 1)
        cv2.rectangle(cv_image, center_center_boundary[0], center_center_boundary[1], (0, 255, 0), 1)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=5)
        font = cv2.FONT_HERSHEY_PLAIN


        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None

        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]),int( M["m01"] / M["m00"]))

            #cv2.putText(cv_image, center.__str__(), (10, 65), font, 2, (0, 255, 0), 2, cv2.LINE_AA)
            #cv2.putText(cv_image, radius.__str__(), (10, 95), font, 2, (0, 255, 0), 2, cv2.LINE_AA)

            if all(i > j for i, j in zip(center,center_center_boundary[0])) and all(i < j for i, j in zip(center,center_center_boundary[1])):
                cv2.putText(cv_image, 'Center', (10, 30), font, 2, (0, 255, 0), 2, cv2.LINE_AA)
                self.pub.publish(1)
            elif all(i > j for i, j in zip(center,top_center_boundary[0])) and all(i < j for i, j in zip(center,top_center_boundary[1])):
                cv2.putText(cv_image, 'Top Center', (10, 30), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
                self.pub.publish(2)
            elif all(i > j for i, j in zip(center, bottom_center_boundary[0])) and all(i < j for i, j in zip(center, bottom_center_boundary[1])):
                cv2.putText(cv_image, 'Bottom Center', (10, 30), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
                self.pub.publish(3)
            elif all(i > j for i, j in zip(center, left_center_boundary[0])) and all(i < j for i, j in zip(center, left_center_boundary[1])):
                cv2.putText(cv_image, 'Left Center', (10, 30), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
                self.pub.publish(4)
            elif all(i > j for i, j in zip(center, right_center_boundary[0])) and all(i < j for i, j in zip(center, right_center_boundary[1])):
                cv2.putText(cv_image, 'Right Center', (10, 30), font, 2, (255, 0, 0), 2, cv2.LINE_AA)
                self.pub.publish(5)
            elif all(i > j for i, j in zip(center, (width/2, 0))):
                if all(i > j for i, j in zip(center, (0, height/2))):
                    cv2.putText(cv_image, 'Right Bottom', (10, 30), font, 2, (255, 255, 255), 2, cv2.LINE_AA)
                    self.pub.publish(6)
                else:
                    cv2.putText(cv_image, 'Right Top', (10, 30), font, 2, (255, 255, 255), 2, cv2.LINE_AA)
                    self.pub.publish(7)
            else:
                if all(i > j for i, j in zip(center, (0, height/2))) :
                    cv2.putText(cv_image, 'Left Bottom', (10, 30), font, 2, (255, 255, 255), 2, cv2.LINE_AA)
                    self.pub.publish(8)
                else:
                    cv2.putText(cv_image, 'Left Top', (10, 30), font, 2, (255, 255, 255), 2, cv2.LINE_AA)
                    self.pub.publish(9)

            if(radius > 10):
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0,255,255), 2)
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        else:
            cv2.putText(cv_image, 'No Ball', (10, 30), font, 2, (0, 0, 255), 2, cv2.LINE_AA)
            self.pub.publish(0)

        #res = cv2.bitwise_and(cv_image, cv_image, mask=mask)


        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1600, 1200)
        cv2.imshow('frame', cv_image)
        #cv2.imshow('mask', mask)
        #cv2.imshow('Res', res)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('ball_teleop')
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Bye')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)