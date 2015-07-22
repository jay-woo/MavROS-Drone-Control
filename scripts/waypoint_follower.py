#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('mavros')

from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from mavros.msg import BatteryStatus, State, OverrideRCIn
from mavros.srv import CommandBool, WaypointPush, WaypointClear, WaypointGOTO, SetMode

from drone import *
import mission_parser

class WaypointFollower():
    def __init__(self, num_drones):
        rospy.init_node('waypoint_follower')
 
        # Joystick variables
        self.axes = []
        self.buttons = []

        # Drone variables
        self.num_drones = num_drones
        self.drones = [Drone(i) for i in xrange(num_drones)]
        self.mode = ['manual' for i in xrange(num_drones)]
        self.waypoints = []

        # ROS publishers
        self.pub_rc = [rospy.Publisher('/drone' + str(i) + '/rc/override', OverrideRCIn, queue_size=10) for i in xrange(num_drones)] 

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.sub_state =   [rospy.Subscriber('/drone' + str(i) + '/state',   State,         self.drones[i].state_callback) for i in xrange(num_drones)]
        self.sub_battery = [rospy.Subscriber('/drone' + str(i) + '/battery', BatteryStatus, self.drones[i].battery_callback) for i in xrange(num_drones)]

        # ROS services
        self.srv_arm =      [rospy.ServiceProxy('/drone' + str(i) + '/cmd/arming', CommandBool) for i in xrange(num_drones)]
        self.srv_mode =     [rospy.ServiceProxy('/drone' + str(i) + '/set_mode', SetMode) for i in xrange(num_drones)]
        self.srv_wp_push =  [rospy.ServiceProxy('/drone' + str(i) + '/mission/push', WaypointPush) for i in xrange(num_drones)]
        self.srv_wp_clear = [rospy.ServiceProxy('/drone' + str(i) + '/mission/clear', WaypointClear) for i in xrange(num_drones)]
        self.srv_wp_goto = [rospy.ServiceProxy('/drone' + str(i) + '/mission/goto', WaypointGOTO) for i in xrange(num_drones)]
       
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
                for i in xrange(self.num_drones):
                    self.srv_arm[i](True)
                print "Arming drones"

            # Disarm drone(s)
            if self.buttons[3]:
                for i in xrange(self.num_drones):
                    self.srv_arm[i](False)
                print "Disarming drones"

            # Push waypoints
            if self.buttons[10]:
                self.waypoints = mission_parser.get_mission(0)
                success = True
                for i in xrange(self.num_drones):
                    res = self.srv_wp_push[i](self.waypoints)
                    success &= res.success
                
                if success:
                    print "Pushed waypoints"
                else:
                    print "Failed to push waypoints"

            # Clear waypoints
            if self.buttons[11]:
                success = True
                for i in xrange(self.num_drones):
                    res = self.srv_wp_clear[i]()
                    success &= res.success

                if success:
                    self.waypoints = []
                    print "Cleared waypoints"
                else:
                    print "Failed to clear waypoints"

            #goto waypoint
            if self.buttons[9]:
                waypoint = mission_parser.make_global_waypoint(42.2933836, -71.2638339)
                success = True
                for i in xrange(self.num_drones):
                    res = self.srv_wp_goto[i](waypoint)
                    success &= res.success
                if success:
                    print "Pushed goto waypoint"
                else:
                    print "Failed goto waypoint"

            #add waypooint
            if self.buttons[8]:
                waypoint = mission_parser.make_global_waypoint(42.2933836, -71.2638339)
                self.waypoints.append(waypoint)
                success = True
                for i in xrange(self.num_drones):
                    res = self.srv_wp_push[i](self.waypoints)
                    success &= res.success
                
                if success:
                    print "Added waypoint"
                else:
                    print "Failed to add waypoint"


            # Begin mission
            if self.buttons[4]:
                self.mode[0] = 'auto'
                self.srv_mode[0](0, '3')
                rc_msg = OverrideRCIn()
                (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[4]) = (1500, 1500, 1250, 1500, 1400) 
                self.pub_rc[0].publish(rc_msg)
                print "Beginning mission"

            # End mission
            if self.buttons[5]:
                self.mode[0] = 'manual'
                print "Ending mission"

        if self.drones[0].armed and self.mode[0] == 'manual':
            rc_msg = OverrideRCIn()
            
            x = 1500 - self.axes[0]*300
            y = 1500 - self.axes[1]*300
            z = 1000 + (self.axes[3]+1)*500
            yaw = 1500 - self.axes[2]*200
    
            (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3]) = (x, y, z, yaw)
            self.srv_mode[0](0, '2')

            self.pub_rc[0].publish(rc_msg)

if __name__ == '__main__':
    try:
        var = WaypointFollower(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
