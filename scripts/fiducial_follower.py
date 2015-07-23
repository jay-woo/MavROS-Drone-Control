#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('mavros')

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from mavros.msg import BatteryStatus, State, OverrideRCIn
from mavros.srv import CommandBool, SetMode

from drone import *

class FiducialFollower():
    def __init__(self):
        rospy.init_node('fiducial_follower')

        # Joystick variables
        self.axes = []
        self.buttons = []

        # Drone variables
        self.drone = Drone(0)

        # Vision variables
        self.fiducial = (0, 0, 0)

        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_vision = rospy.Subscriber('/fiducial', Point, self.fiducial_callback)

        self.sub_state = rospy.Subscriber('/drone/state', State, self.drone.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.drone.battery_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

    def fiducial_callback(self, data):
        self.fiducial = (data.x, data.y, data.z)

    def fly(self):
        if self.buttons:
            # Arm drone
            if self.buttons[2]:
                self.srv_mode(0, '5')
                self.srv_arm(True)
                print "Arming drone"

            # Disarm drone
            if self.buttons[3]:
                self.srv_arm(False)
                print "Disarming drone"

        if self.drone.armed:
            rc_msg = OverrideRCIn()

            if abs(self.axes[0]) > 0.00001 or abs(self.axes[1]) > 0.00001:
                x = 1500 - self.axes[0] * 300
                y = 1500 - self.axes[1] * 300
                yaw = 1500 - self.axes[2] * 200
            else:
                scale_x = self.fiducial[0] 
                scale_y = self.fiducial[1]
 
                x = 1500 + scale_x * 50
                y = 1500 - scale_y * 50
                yaw = 1500

            if self.axes[3] > 0.8:
                z = 1600
            elif self.axes[3] < -0.8:
                z = 1400
            else:
                z = 1500

            (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[6]) = (x, y, z, yaw, 1250)
            self.pub_rc.publish(rc_msg)

if __name__ == '__main__':
    try:
        var = FiducialFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
