#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('mavros')

from sensor_msgs.msg import Joy
from mavros.msg import BatteryStatus, State, OverrideRCIn
from mavros.srv import CommandBool, SetMode

from drone import *
from find_fiducial import *

class FiducialFollower():
    def __init__(self):
        rospy.init_node('fiducial_follower')

        # Joystick variables
        self.axes = []
        self.buttons = []

        # Drone variables
        self.drone = Drone(0)
        self.mode = 'manual'
        
        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.sub_state = rospy.Subscriber('/drone/state', State, self.drone.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.drone.battery_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)

        # Initial drone setup
        self.srv_mode(0, '5')

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

    def fly(self):
        if self.buttons:
            # Arm drone
            if self.buttons[2]:
                self.srv_arm(True)
                print "Arming drone"

            # Disarm drone
            if self.buttons[3]:
                self.srv_arm(False)
                print "Disarming drone"

            # Track fiducial (drone must be in the air first!)
            if self.buttons[4]:
                self.mode = 'auto'

            # Stop tracking fiducial
            if self.buttons[5]:
                self.mode = 'manual'

        if self.drone.armed:
            rc_msg = OverrideRCIn()

            if self.mode == 'manual':
                x = 1500 - self.axes[0] * 300
                y = 1500 - self.axes[1] * 300
                z = 1000 + (self.axes[3]+1) * 500
                yaw = 1500 - self.axes[2] * 200

            if self.mode == 'auto':
                x = 1550
                y = 1500
                z = 1500
                yaw = 1500

            (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[4]) = (x, y, z, yaw, 1500)
            self.pub_rc.publish(rc_msg)

if __name__ == '__main__':
    try:
        var = FiducialFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
