#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib as plt
import math
import rospy

from std_msgs.msg import UInt8
from geometry_msgs.msg import Point

from calibrate import *

class Vision():
    def __init__(self):
        self.cap = cv2.VideoCapture(1)
        self.img = None
        self.canny = None
        self.mode = 0

        rospy.init_node('camera')

        self.sub_mode = rospy.Subscriber('mode', UInt8, self.mode_callback)
        self.pub_fiducial = rospy.Publisher('vision', Point, queue_size=10)

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.mode == 3:
                self.track_object()
            elif self.mode == 5:
                self.find_squares()

            if self.img != None:
                cv2.imshow('camera', self.img)
                cv2.imshow('canny', self.canny)
                cv2.waitKey(1)

            r.sleep()

    def mode_callback(self, data)
        self.mode = data

    def track_object(self):
        pass

    def find_squares(self):
        ret, img = self.cap.read()
        img = calibrate(img)
        img_display = img
        img = cv2.inRange(img, np.array([50, 50, 50], dtype=np.uint8), np.array([255, 255, 255], dtype=np.uint8))
        img = cv2.GaussianBlur(img, (5,5), 0)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, (9,9))
    
        squares = []
        bin = cv2.Canny(img, 200, 250, apertureSize=5)
        self.canny = bin
        bin = cv2.dilate(bin, None)
        retval, bin = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        i1 = 0
        for cnt in contours:
            children = []
            children_final = []
            children_areas = 0
            average_area = 0.0
            cnt_len = cv2.arcLength(cnt, True)
            cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
            if len(hierarchy[0]) > 0:
                i2 = hierarchy[0][i1][2]
                while i2 != -1:
                    children.append(contours[i2])
                    children_areas += cv2.contourArea(contours[i2])
                    i2 = hierarchy[0][i2][0]
            i1 += 1
    
            if len(children) > 0:
                average_area = float(children_areas) / len(children)
                for cld in children:
                    if abs(cv2.contourArea(cld) - average_area) < 100:
                        children_final.append(cld)
    
            if len(cnt) >= 4 and cv2.isContourConvex(cnt) and len(children_final) >= 5:
                cnt = cnt.reshape(-1, 2)
                max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                if max_cos < 0.1: 
                    squares.append(cnt) 
    
                if len(squares) == 2:
                    if cv2.contourArea(squares[0]) > cv2.contourArea(squares[1]):
                        squares.pop(0)
                    else:
                        squares.pop(1)
    
        if len(squares) != 0:
            cv2.drawContours( img_display, squares, -1, (0, 255, 0), 2 )

            M = cv2.moments(np.array(squares))
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])
            z = cv2.contourArea(squares[0])
               
            fiducial_msg = Point()
            (fiducial_msg.x, fiducial_msg.y, fiducial_msg.z) = (x, y, z)
            self.pub_fiducial.publish(fiducial_msg)

        self.img = img_display
 
    def angle_cos(self, p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

if __name__ == '__main__':
    try:
        var = Vision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
