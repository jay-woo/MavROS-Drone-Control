import cv2
import rospy
from gopro_calibration import *

def calibrate(img):
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]

    return dst
