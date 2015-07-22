#!/usr/bin/env python
import rospy
import roslib
import time
import math
roslib.load_manifest('mavros')

from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float64, UInt8
from sensor_msgs.msg import Joy, NavSatFix
from mavros.msg import BatteryStatus, State, OverrideRCIn, Waypoint
from mavros.srv import CommandBool, WaypointPush, WaypointClear, WaypointGOTO, SetMode

from drone import *
import mission_parser

class Snotbot():
    ###
    # Initializes drone control variables
    ###
    def __init__(self):
        # Initializes ROS node
        rospy.init_node('snotbot')

        # Joystick variables
        self.axes = []
        self.buttons = []

        # Drone variables
        self.drone = Drone(0)  # Drone class contains valuable functions/variables

        # Vision variables
        self.fiducial = [-1, -1, -1]

        # Miscellaneous variables
        self.time_mode_started = 0    # Records when each mode in the finite state machine started
        self.target_alt = 5.0
        self.failsafe = False
        self.launched = False

        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)
        self.pub_rc = rospy.Publisher('mode', UInt8, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_vision = rospy.Subscriber('/vision', Point, self.vision_callback)

        self.sub_state = rospy.Subscriber('/drone/state', State, self.drone.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.drone.battery_callback)
        self.sub_drone_gps = rospy.Subscriber('/drone/gps/fix', NavSatFix, self.drone.gps_callback)
        self.sub_altitude = rospy.Subscriber('/drone/global_position/rel_alt', Float64, self.drone.altitude_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)
        self.srv_wp_push =  rospy.ServiceProxy('/drone/mission/push', WaypointPush)
        self.srv_wp_clear = rospy.ServiceProxy('/drone/mission/clear', WaypointClear)
        self.srv_wp_goto = rospy.ServiceProxy('/drone/mission/goto', WaypointGOTO)

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    ###
    # Retrieves joystick data
    ###
    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

    ###
    # Retrieves vision data
    ###
    def vision_callback(self, data):
       self.fiducial = [data.x, data.y, data.z]

    ########################
    # FINITE STATE MACHINE #
    ########################

    ###
    # Mode 1: arms the drone
    ###
    def arm(self):
        self.drone.z = 1000

        # Arming process
        if not self.drone.armed:
            self.srv_mode(0, '5') # Loiter
            self.srv_arm(True)

        # Waits a few seconds before switching to launch mode
        if millis() - self.time_mode_started > 3000:
            self.time_mode_started = millis()
            self.srv_mode(0, '3')
            self.drone.mode = 2

    ###
    # Mode 2: launches the drone to a certain altitude
    ###
    def launch(self):
        self.drone.z = 1250

        # Launches the drone at the current location
        if not self.launched:
            waypoints = mission_parser.takeoff_waypoints(self.target_alt)           
            self.srv_wp_push(waypoints)
            self.launched = True

        if abs(self.drone.altitude - self.target_alt) < 0.5
            self.time_mode_started = millis()
            self.srv_mode(0, '5')
            self.drone.mode = 3

    ###
    # Mode 3: waits for user to select object to track
    ###
    def track(self):
        self.drone.z = 1500
        self.mode = 4

    ###
    # Mode 4: return to launch
    ###
    def rtl(self):
        self.mode = 5

    ###
    # Mode 5: land
    ###

    def land(self):
        self.mode = 6

    ###
    # Mode 6: disarm
    ###
    def disarm(self):
        self.srv_arm(False)

    ###
    # Publishes RC commands to the vehicle, depending on what mode it is currently in
    ###
    def fly(self): 
        if self.buttons:
            # Button 1 - enters failsafe mode (enables full control)
            if self.buttons[0]:
                self.failsafe = not self.failsafe 

            # Button 3 - arms the drone
            if self.buttons[2]:
                self.srv_arm(True)
                print "Arming drone"

            # Button 4 - disarms drone
            if self.buttons[3]:
                self.srv_arm(False) 
                print "Disarming drone"

            # Button 5 - initiate autonomy routine
            if self.buttons[4]:
                self.drone.mode = 1
                self.time_mode_started = millis()
                print "Beginning finite state machine"
                
            # Button 6 - end autonomy routine (RTL)
            if self.buttons[5]:
                self.drone.mode = 5
                print "Returning to launch and landing"

        # Initiates finite state machines
        if not self.failsafe:
            # Arming
            if self.drone.mode == 1:
                self.srv_mode(0, '5')
                self.arm()

            # Launching
            if self.drone.mode == 2:
                self.launch()

            # Tracking
            if self.drone.mode == 3:
                self.track()

            # Returning to launch
            if self.drone.mode == 4:
                self.rtl()

            # Landing
            if self.drone.mode == 5:
                self.land()

            # Disarming
            if self.drone.mode == 6:
                self.disarm()

        else:
            # Reads joystick values
            self.srv_mode(0, '5')
            self.drone.mode = 0

            x = 1500 - self.axes[0] * 300
            y = 1500 - self.axes[1] * 300
            z = 2000 + self.axes[3] * 1000
            yaw = 1500 - self.axes[2] * 300

            (self.drone.x, self.drone.y, self.drone.z, self.drone.yaw) = (x, y, z, yaw)

        # Publishes commands
        if self.drone.armed:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [self.drone.x, self.drone.y, self.drone.z, self.drone.yaw, 1400, 0, 0, self.drone.cam_tilt] 
            self.pub_rc.publish(rc_msg) 

        self.sub_mode.publish(self.drone.mode)
 
###
# Calculates the number of milliseconds since the epoch
###
def millis():
    return int(round(time.time() * 1000))

if __name__ == '__main__':
    try:
        var = Snotbot()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
