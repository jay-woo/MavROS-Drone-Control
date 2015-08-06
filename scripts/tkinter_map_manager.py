#!/usr/bin/env python
import rospy
import urllib2
import cv2
import numpy as np
from geometry_msgs.msg import Polygon, Point32
import roslib
roslib.load_manifest('drone_control')
from drone_control.msg import mission
import PIL.Image
import PIL.ImageTk
import copy
import math

import Tkinter as tk
#import ttk

#set things here
WINDOW_NAME = "Map"
LAT = 42.293173 #for oval #42.290447 #for rugby field
LON = -71.263540 #for oval #-71.265311 #for rugby field
MAP_HEIGHT = 640 #don't change
ZOOM = 19 #increasing by 1 halves the linear dimensions of the map


class MapManager(object): #created by the mapGUI class (below)
    '''
    This class containes all of the code associated with things appearing on the map
    this includes the actuall coordinates of the waypoitns
    '''
    #URL for map image
    BASE_URL = "http://maps.googleapis.com/maps/api/staticmap"

    def __init__(self, map_height, zoom, lat, lon):
        self.map_height = map_height
        self.zoom = zoom
        self.static_map = self.make_map_request(lat, lon)
        self.img = copy.copy(self.static_map) #copy image so it can be reset to original
        self.center_lat = lat
        self.center_lon = lon
        self.plotted_points = [] #contains the lat-lon of all waypoints

    def make_map_request(self, lat, lon):
        '''
        gets map image from google and returns the image as an openCV arrray
        '''
        lat = "%s" % lat
        lon = "%s" % lon
        params = (self.BASE_URL, lat, lon, self.zoom, self.map_height, self.map_height)
        full_url = "%s?center=%s,%s&zoom=%s&size=%sx%s&sensor=false&maptype=satellite" % params
        response = urllib2.urlopen(full_url)
        png_bytes = np.asarray([ord(char) for char in response.read()], dtype=np.uint8)
        cv_array = cv2.imdecode(png_bytes, cv2.CV_LOAD_IMAGE_UNCHANGED)
        return cv_array

    @property
    def degrees_in_map(self):
        '''
        gets degrees in map by converting from meters to degrees
        Accounts for longitude size by calculating the size of a degree of longitude
        at the center latitude, then treats all longitude in the map as equal to that
        '''
        deg_lat = self.linear_meters_in_map/111319.9
        scale = math.cos(self.center_lat*math.pi/180)
        deg_lon = deg_lat/scale
        return (deg_lat, deg_lon)

    @property
    def linear_meters_in_map(self):
        '''
        Uses an arbitrary number which is the number of meters in one scale of google maps
        converts from that scale to the input zoom based on the linear meters being cut
        in half for each increase in zoom level
        '''
        meters_in_map = 591657550.500000 / pow(2, self.zoom+3)
        return meters_in_map

    def _window_x_y_to_grid(self, x, y):
        '''
        converts graphical x, y coordinates to grid coordinates
        where (0, 0) is the very center of the window
        '''
        center_x = center_y = self.map_height / 2
        new_x = x - center_x
        new_y = -1 * (y - center_y)
        return new_x, new_y

    def _grid_x_y_to_window(self, x, y):
        center_x = center_y = self.map_height / 2
        new_x = center_x + x
        new_y = center_y - y
        return new_x, new_y

    def x_y_to_lat_lon(self, x, y):
        grid_x, grid_y = self._window_x_y_to_grid(x, y)
        offset_x_degrees = (float(grid_x) / self.map_height) * self.degrees_in_map[1]
        offset_y_degrees = (float(grid_y) / self.map_height) * self.degrees_in_map[0]
        # lat = y, lon = x
        return self.center_lat + offset_y_degrees, self.center_lon + offset_x_degrees

    def lat_lon_to_x_y(self, lat, lon):
        '''
        Returns x, y coordinates where (0, 0) is the top left
        '''
        offset_lat_degrees = lat - self.center_lat
        offset_lon_degrees = lon - self.center_lon
        grid_x = (offset_lon_degrees / self.degrees_in_map[1]) * self.map_height
        grid_y = (offset_lat_degrees / self.degrees_in_map[0]) * self.map_height
        window_x, window_y = self._grid_x_y_to_window(grid_x, grid_y)
        return int(window_x), int(window_y)

    def add_waypoint(self, event):
        #add a waypoint
        x = event.x
        y = event.y
        lat, lon = self.x_y_to_lat_lon(x, y)
        self.plot_point(lat, lon)
        return lat, lon
    def rm_waypoint(self, event):
        #remove a waypoint
        self.plotted_points.pop(-1)
        self.img = copy.copy(self.static_map)

    def plot_point(self, lat, lon):
        self.plotted_points.append([lat, lon])

    def get_plotted_points_as_x_y_list(self):
        #returns plotted lat, lon points as drawable (x, y) window coordinates
        return [self.lat_lon_to_x_y(point[0], point[1]) for point in self.plotted_points]


class MapGui():
    '''
    This class runs the tkinter window, managing all the buttons and
    publishing messages to ROS when required
    '''
    def __init__(self):
        #colors for the map icons
        self.RED = cv2.cv.Scalar(0, 0, 255)
        self.GREEN = cv2.cv.Scalar(0, 255, 0)
        self.YELLOW = cv2.cv.Scalar(0, 180, 180)

        #set initial values for variables
        self.do_takeoff = True
        self.do_rtl = True
        self.hold_forever = False
        self.current_waypoint = 0
        self.old_points = 'unset'

        #ros setup
        self.pub = rospy.Publisher('map_waypoints', mission, queue_size=10)
        rospy.init_node('tkinter_map')

        #setup window
        self.root = tk.Tk() #base tkinter window
        setattr(self.root, 'quit_flag', False) #make the  quit the program
        self.root.protocol('WM_DELETE_WINDOW', self.set_quit_flag)

        #setup map tile centered at (lat, lon)
        self.start_map(LAT, LON) #input (lat, lon)

        #set the text for the 'current waypoint' label
        self.waypoint_text = tk.StringVar()
        self.waypoint_text.set('current waypoint: %d' %self.current_waypoint)

        #setup buttons
        self.buttons()

        #run everything
        self.root.mainloop()

    def set_quit_flag(self):
        #function to quit program on exit
        self.root.quit_flag = True

    # initialize the map
    def start_map(self, lat, lon):
        self.map_label = tk.Label(self.root)  #label contains the map
        self.map_label.pack()
        self.map = MapManager(MAP_HEIGHT, ZOOM, lat, lon) #create a MapManager (from above)
        self.root.after(0, func=self.loop)
        self.map_label.bind("<Button-1>", self.add_point) #make left clicking add a waypoint
        self.map_label.bind("<Button-3>", self.rm_point) #make right clicking remove a waypoint

    def add_point(self, event):
        #add a waypoint
        self.map.add_waypoint(event)
        self.map.plotted_points[-1].append(self.alt_entry.get())
        self.current_waypoint = len(self.map.plotted_points)
        self.waypoint_text.set('current waypoint: %d' %self.current_waypoint)

    def rm_point(self, event):
        #remove a waypoint
        self.map.rm_waypoint(event)
        self.current_waypoint = len(self.map.plotted_points)
        self.waypoint_text.set('current waypoint: %d' %self.current_waypoint)

    def next_point(self):
        #toggle to next waypoint and reset displays
        if self.current_waypoint < len(self.map.plotted_points):
            self.current_waypoint += 1
            self.waypoint_text.set('current waypoint: %d' %self.current_waypoint)
            self.alt_entry.delete(0, tk.END)
            self.alt_entry.insert(0, str(self.map.plotted_points[self.current_waypoint-1][2]))

    def prev_point(self):
        #toggle to previous waypoint and reset displays
        if self.current_waypoint > 1:
            self.current_waypoint -= 1
            self.waypoint_text.set('current waypoint: %d' %self.current_waypoint)
            self.alt_entry.delete(0, tk.END)
            self.alt_entry.insert(0, str(self.map.plotted_points[self.current_waypoint-1][2]))


    def click_hold(self):
        #toggle whether to hold at the end of the mission
        #ensure that hold forever and rtl are not both set
        self.hold_forever = not self.hold_forever
        if self.do_rtl and self.hold_forever:
            self.rtl_end.toggle()
            self.do_rtl = not self.do_rtl

    def click_rtl(self):
        #toggle whether to rtl at the end of the mission
        #ensure that rtl and hold forever are not both set
        self.do_rtl = not self.do_rtl
        if self.do_rtl and self.hold_forever:
            self.hold_end.toggle()
            self.hold_forever = not self.hold_forever

    def click_takeoff(self):
        #toggle whether to takeoff at the beginning of the mission
        self.do_takeoff = not self.do_takeoff

    def change_alt(self, event):
        #handle changing the set altitude of a waypoint
        #(called on enter while altitude entry box is active)
        if self.current_waypoint > 0:
            new_alt = self.alt_entry.get()
            index = self.current_waypoint-1
            self.map.plotted_points[index][2] = new_alt

    def buttons(self):
        #setup the display below the map
        upper_bar = tk.Frame(self.root, height=100)

        self.hold_end = tk.Checkbutton(upper_bar, text='hold at last waypoint', command=self.click_hold)
        self.hold_end.grid(row=0, column=7, sticky='w')

        self.takeoff_start = tk.Checkbutton(upper_bar, text='start with takeoff', command=self.click_takeoff)
        self.takeoff_start.select() #default to selected
        self.takeoff_start.grid(row=0, column=6, padx=10, sticky='w')

        self.rtl_end = tk.Checkbutton(upper_bar, text='end with RTL', command=self.click_rtl) 
        self.rtl_end.select()
        self.rtl_end.grid(row=1, column=6, padx=10, sticky='w')

        self.alt_entry = tk.Entry(upper_bar, width=3)
        self.alt_entry.insert(0, '10')
        self.alt_entry.bind('<Return>', self.change_alt)
        self.alt_entry.grid(row=0, column=5, rowspan=2)
        alt_label = tk.Label(upper_bar, text='set alt')
        alt_label.grid(row=0, column=4, rowspan=2)

        space = tk.Label(upper_bar, text=' ', width=1)
        space.grid(row=0, column=3)

        next_waypoint = tk.Button(upper_bar, text='>>', width=4, command=self.next_point)
        #next_waypoint.pack(side=tk.RIGHT, padx=5, pady=5)
        next_waypoint.grid(row=0, column=2, rowspan=2)
        last_waypoint = tk.Button(upper_bar, text='<<', width=4, command=self.prev_point)
        #last_waypoint.pack(side=tk.RIGHT, padx=5, pady=5)
        last_waypoint.grid(row=0, column=1, rowspan=2)

        cur_waypoint = tk.Label(upper_bar, textvariable=self.waypoint_text, padx=10)
        #cur_waypoint.pack(side=tk.RIGHT, padx=5, pady=5)
        cur_waypoint.grid(row=0, column=0, rowspan=2)

        upper_bar.pack(fill='x')

        save_mission_button = tk.Button(self.root, text='Save to Mission', command=self.save_mission)
        save_mission_button.pack(side=tk.RIGHT, padx=5, pady=5)

        save_guided_button = tk.Button(self.root, text='Save to Guided', command=self.save_guided)
        save_guided_button.pack(side=tk.RIGHT, padx=5, pady=5)

        save_bound_button = tk.Button(self.root, text='Save as Boundary', command=self.save_bound)
        save_bound_button.pack(side=tk.RIGHT, padx=5, pady=5)

    def loop(self):
        #main loop program
        if self.root.quit_flag:
            self.root.destroy()  # this avoids the update event being in limbo
        else:
            self.update()
            self.root.after(10, func=self.loop)

    def update(self):
        #update the map, runs continuously
        points = self.map.get_plotted_points_as_x_y_list() #get window coorinates of waypoints
        if self.old_points == 'unset' or points != self.old_points: #run in waypoints have changed
            self.old_points = points
            for i in range(len(points)): #update wayoint markers
                if i > 0:
                    cv2.line(self.map.img, pt1=points[i-1], pt2=points[i], color=self.YELLOW, thickness = 2)
                    cv2.circle(self.map.img, center=points[i], radius=5, color=self.RED, thickness=-1)
                else:
                    cv2.circle(self.map.img, center=points[i], radius=5, color=self.GREEN, thickness=-1)
            #convert from openCV image to tkinter image
            rgb_img = cv2.cvtColor(self.map.img, cv2.COLOR_BGR2RGB)
            pil_image = PIL.Image.fromarray(rgb_img)
            tk_image = PIL.ImageTk.PhotoImage(image=pil_image)
            self.map_label.configure(image=tk_image)

            self.map_label._image_cache = tk_image  # avoid garbage collection
            self.root.update()


    #functions for sending waypoints to the rest of the code
    def save_guided(self):
        self.publish('guided')

    def save_bound(self):
        self.publish('bounds')

    def save_mission(self):
        self.publish('mission')

    def publish(self, dest):
        msg = mission()
        msg.waypoint.points=[]
        for location in self.map.plotted_points:
            point = Point32()
            point.x = location[0]
            point.y = location[1]
            point.z = int(location[2])

            msg.waypoint.points.append(point)

        msg.takeoff = self.do_takeoff
        msg.rtl = self.do_rtl
        msg.hold_forever = self.hold_forever

        msg.dest = dest

        self.pub.publish(msg)

if __name__ == '__main__':
    MapGui()