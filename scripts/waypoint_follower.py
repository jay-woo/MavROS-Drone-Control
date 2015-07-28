#!/usr/bin/env python
import rospy
import roslib
import time
import rospkg
roslib.load_manifest('mavros')

import os
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from mavros.msg import BatteryStatus, State, OverrideRCIn, Waypoint
from geometry_msgs.msg import Polygon
from mavros.srv import CommandBool, WaypointPush, WaypointClear, WaypointGOTO, SetMode, WaypointSetCurrent

from drone import *
import mission_parser

DEFAULT_ALT = 10

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
        self.guided_waypoints = [[42.2933519, -71.2639090], [42.2933836, -71.2638339]]
        self.guided_index = 0
        self.google_waypoints = []

        # ROS publishers
        self.pub_rc = [rospy.Publisher('/drone' + str(i) + '/rc/override', OverrideRCIn, queue_size=10) for i in xrange(num_drones)] 

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_google_points = rospy.Subscriber('/google_waypoints', Polygon, self.google_points_callback)

        self.sub_state =   [rospy.Subscriber('/drone' + str(i) + '/state',   State,         self.drones[i].state_callback) for i in xrange(num_drones)]
        self.sub_battery = [rospy.Subscriber('/drone' + str(i) + '/battery', BatteryStatus, self.drones[i].battery_callback) for i in xrange(num_drones)]

        # ROS services
        self.srv_arm =      [rospy.ServiceProxy('/drone' + str(i) + '/cmd/arming', CommandBool) for i in xrange(num_drones)]
        self.srv_mode =     [rospy.ServiceProxy('/drone' + str(i) + '/set_mode', SetMode) for i in xrange(num_drones)]
        self.srv_wp_push =  [rospy.ServiceProxy('/drone' + str(i) + '/mission/push', WaypointPush) for i in xrange(num_drones)]
        self.srv_wp_clear = [rospy.ServiceProxy('/drone' + str(i) + '/mission/clear', WaypointClear) for i in xrange(num_drones)]
        self.srv_wp_goto = [rospy.ServiceProxy('/drone' + str(i) + '/mission/goto', WaypointGOTO) for i in xrange(num_drones)]
        self.srv_set_current = [rospy.ServiceProxy('/drone' + str(i) + '/mission/set_current', WaypointSetCurrent) for i in xrange(num_drones)]
       
        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    #ros callback functions
    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

    def google_points_callback(self, data):
        self.google_waypoints = [[point.x, point.y] for point in data.points]


    #functions for map gui
    def launch_map(self):
        os.system('gnome-terminal -x ../catkin_ws/src/MavROS-Drone-Control/scripts/start_map.sh')

    def set_guided_to_map(self):
        self.guided_waypoints = self.google_waypoints
        self.guided_index = 0
        print "guided points reset"

    def set_mission_to_map(self):
        self.waypoints = [self.make_global_waypoint(point[0], point[1]) for point in self.google_waypoints]
        self.start_with_takeoff()
        self.end_with_rtl()
        self.push_waypoints()


    #guided mode functions
    def start_guided(self):
        self.mode[0] = 'guided'
        self.srv_mode[0](0, '4')
        self.guided_index = 0
        print "guided mode started"

    def set_guided_waypoint(self, lat, lon, alt=DEFAULT_ALT):
        waypoint = self.make_global_waypoint(lat, lon, alt)
        waypoint.is_current = 2
        res = self.srv_wp_push[0]([waypoint])
        if res.success:
            print "set waypoint"
        else:
            print "waypoint failed"


    #mission functions
    def continue_mission(self):
        ''' make drone hold position forever at end of mission instead of land '''
        final_lat = self.waypoints[-1].x_lat
        final_lon = self.waypoints[-1].y_long
        hold_pos = self.make_global_waypoint(final_lat, final_lon)
        hold_pos.command = 17
        self.waypoints.append(hold_pos)

    def end_with_rtl(self):
        rtl = Waypoint()
        rtl.command = 20
        self.waypoints.append(rtl)

    def start_with_takeoff(self):
        takeoff = Waypoint()
        takeoff.command = 22
        takeoff.z_alt = DEFAULT_ALT
        self.waypoints.insert(0, takeoff)

    def make_global_waypoint(self, lat, lon, alt=DEFAULT_ALT, hold=5):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drone_control')
        waypoint = Waypoint()

        waypoint.frame = 3
        waypoint.command = 16
        waypoint.is_current = 0
        waypoint.autocontinue = False
        waypoint.param1 = hold #hold time
        waypoint.param2 = 2
        waypoint.param3 = 0
        waypoint.param4 = 0
        waypoint.x_lat = lat
        waypoint.y_long = lon
        waypoint.z_alt = alt

        return waypoint

    def restart_mission(self):
        success = True
        for i in xrange(self.num_drones):
            self.srv_set_current[i](0)
        print "restarted mission"


    #button action functions
    def guided_function(self):
        if self.mode[0] == 'guided':
            if self.guided_index >= len(self.guided_waypoints):
                self.RTL()
            else:
                waypoint = self.guided_waypoints[self.guided_index]
                self.set_guided_waypoint(waypoint[0], waypoint[1])
                self.guided_index += 1
        else:
            self.start_guided()

    def RTL(self):
        self.mode[0] = 'RTL'
        self.srv_mode[0](0, '6')
        print 'returning to launch'

    def push_waypoints(self):
        success = True
        for i in xrange(self.num_drones):
            res = self.srv_wp_push[i](self.waypoints)
            success &= res.success
        if success:
            print "Pushed waypoints"
        else:
            print "Failed to push waypoints"
        return success

    def clear_waypoints(self):
        success = True
        for i in xrange(self.num_drones):
            res = self.srv_wp_clear[i]()
            success &= res.success

        if success:
            self.waypoints = []
            print "Cleared waypoints"
        else:
            print "Failed to clear waypoints"

    def add_waypoints(self, coordinates):
        new_waypoints = [self.make_global_waypoint(lat, lon) for [lat, lon] in coordinates]
        self.waypoints.extend(new_waypoints)
        #self.continue_mission() #don't land at end of mission
        self.push_waypoints()

    def begin_mission(self):
        self.mode[0] = 'auto'
        self.srv_mode[0](0, '3')
        rc_msg = OverrideRCIn()
        (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3], rc_msg.channels[4]) = (1500, 1500, 1250, 1500, 1400) 
        self.pub_rc[0].publish(rc_msg)
        print "Beginning mission"

    def end_mission(self):
        self.mode[0] = 'manual'
        self.srv_mode[0](0, '5')
        print "Ending mission"

    def arm(self):
        for i in xrange(self.num_drones):
            self.srv_arm[i](True)
        print "Arming drones"

    def disarm(self):
        for i in xrange(self.num_drones):
            self.srv_arm[i](False)
        print "Disarming drones"


    #set joystic button commands
    #note code button numbers = joystick buttons - 1
    def fly(self):
        if self.buttons:
            #RTL
            if self.buttons[0]:
                self.RTL()

            # Arm drone(s)
            if self.buttons[2]:
                self.arm()

            # Disarm drone(s)
            if self.buttons[3]:
                self.disarm()

            #set mission
            if self.buttons[10]:
                self.waypoints = mission_parser.get_mission(0)
                #self.continue_mission() #keep drone from landing at end of mission
                success = self.push_waypoints() #push to drone

            if self.buttons[11]:
                self.clear_waypoints()

            '''
            #set new waypoints - in flight, ignore existing waypoints and start new mission
            if self.buttons[9]:
                self.waypoints = mission_parser.get_mission(1)
                #self.continue_mission()
                self.push_waypoints()
                self.restart_mission()
            '''

            #save map points to mission
            if self.buttons[9]:
                self.set_mission_to_map()

            #add waypoints
            if self.buttons[8]:
                self.add_waypoints([[42.293394, -71.263916],[42.293357, -71.263997]])

            #enter guided mode/toggle guided waypoints
            if self.buttons[1]:
                self.guided_function()

            if self.buttons[4]:
                self.begin_mission()

            if self.buttons[5]:
                self.end_mission()

            if self.buttons[6]:
                self.set_guided_to_map()

            if self.buttons[7]:
                self.launch_map()

            time.sleep(0.01)


        #joystick control for manual
        if self.drones[0].armed and self.mode[0] == 'manual':
            rc_msg = OverrideRCIn()
            
            x = 1500 - self.axes[0]*300
            y = 1500 - self.axes[1]*300
            z = 1000 + (self.axes[3]+1)*500
            yaw = 1500 - self.axes[2]*200
            
            (rc_msg.channels[0], rc_msg.channels[1], rc_msg.channels[2], rc_msg.channels[3]) = (x, y, z, yaw)

            self.pub_rc[0].publish(rc_msg)

if __name__ == '__main__':
    try:
        var = WaypointFollower(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
