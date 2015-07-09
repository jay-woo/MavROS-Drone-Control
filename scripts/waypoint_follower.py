#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('mavros')

from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from mavros.msg import BatteryStatus, State, OverrideRCIn
from mavros.srv import CommandBool, WaypointPush, WaypointClear

from drone import *
import mission_parser

class WaypointFollower():
    def __init__(self, num_drones):
        rospy.init_node('waypoint_follower')
 
        # Joystick variables
        self.axes = []
        self.buttons = []

        # Drone variables
        self.drones = [Drone(i) for i in xrange(num_drones)]

        # ROS publishers
        self.pub_rc = [rospy.Publisher('/drone' + str(i) + '/rc/override', OverrideRCIn, queue_size=10) for i in xrange(num_drones)] 

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.sub_state = [rospy.Subscriber('/drone' + str(i) + '/state', State, self.drones[i].state_callback) for i in xrange(num_drones)]
        self.sub_battery = [rospy.Subscriber('/drone' + str(i) + '/battery', BatteryStatus, self.drones[i].battery_callback) for i in xrange(num_drones)]

        # ROS services
        self.srv_arm = [rospy.ServiceProxy('/drone' + str(i) + '/cmd/arming', CommandBool) for i in xrange(num_drones)]
        self.srv_wp_push = [rospy.ServiceProxy('/drone' + str(i) + '/mission/push', WaypointPush) for i in xrange(num_drones)]
        self.srv_wp_clear = [rospy.ServiceProxy('/drone' + str(i) + '/mission/clear', WaypointClear) for i in xrange(num_drones)]
       
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
            # Arm drone(s)
            if self.buttons[2]:
                self.srv_arm0(True)
                self.srv_arm1(True)
                print "Arming drones"
            # Disarm drone(s)
            if self.buttons[3]:
                self.srv_arm0(False)
                self.srv_arm1(False)
                print "Disarming drones"

        if self.drones[0].armed or self.drones[1].armed:
            x = 1500 - self.axes[0]*300
            y = 1500 - self.axes[1]*300
            z = 1000 + (self.axes[3]+1)*500
            yaw = 1500 - self.axes[2]*300
    
            rc_msg = OverrideRCIn()
            (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3]) = (x, y, z, yaw)
    
            if self.drones[0].armed:
                self.pub_rc0.publish(rc_msg)
            if self.drones[1].armed:
                self.pub_rc1.publish(rc_msg)

    def start_mission0(self):
        waypoints = mission_parser.get_mission()
        print self.srv_wp_push0(waypoints)

    def start_mission1(self):
        waypoints = mission_parser.get_mission()
        self.srv_wp_push1(waypoints)

    def end_mission0(self):
        self.srv_wp_clear0()

    def end_mission1(self):
        self.srv_wp_clear1()

if __name__ == '__main__':
    try:
        var = WaypointFollower(2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
