#!/usr/bin/env python
import urllib2
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Polygon, Point32
import copy
import math

WINDOW_NAME = "Map"
MAP_HEIGHT = 640
ZOOM = 19


class MapManager(object):

    BASE_URL = "http://maps.googleapis.com/maps/api/staticmap"

    def __init__(self, map_height, zoom, lat, lon):
        self.map_height = map_height
        self.zoom = zoom
        self.static_map = self.make_map_request(lat, lon)
        self.img = copy.copy(self.static_map)
        self.center_lat = lat
        self.center_lon = lon
        self.plotted_points = []

    def make_map_request(self, lat, lon):
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
        This logic is based on the idea that zoom=0 returns 360 degrees
        '''
        #return (self.map_height / 256.0) * (360.0 / pow(2, self.zoom))
        deg_lat = self.linear_meters_in_map/111319.9
        scale = math.cos(self.center_lat*math.pi/180)
        deg_lon = deg_lat/scale
        return (deg_lat, deg_lon)


    def degrees_to_meters(self, degrees):
        equator_length_km = 40008
        km_per_degree = equator_length_km / 360.0
        m_per_degree = km_per_degree * 1000
        return degrees * m_per_degree

    @property
    def linear_meters_in_map(self):

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

    def mouse_callback(self, event, x, y, flag=0, param=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            lat, lon = self.x_y_to_lat_lon(x, y)
            self.plot_point(lat, lon)
            print lat, lon
        if event == cv2.EVENT_RBUTTONDOWN:
            self.plotted_points.pop(-1)
            self.img = copy.copy(self.static_map)


    def plot_point(self, lat, lon):
        self.plotted_points.append((lat, lon,))

    def get_plotted_points_as_x_y_list(self):
        '''
        returns plotted lat, lon points as drawable (x, y) window coordinates
        '''
        return [self.lat_lon_to_x_y(*tuple_inst) for tuple_inst in self.plotted_points]

# initialize the map
def start_map():
    starting_coords = (42.293173, -71.263840)  # my apartment...
    manager = MapManager(MAP_HEIGHT, ZOOM, starting_coords[0], starting_coords[1])

    # BGR colors
    RED = cv2.cv.Scalar(0, 0, 255)
    YELLOW = cv2.cv.Scalar(0, 180, 180)
    cv2.namedWindow(WINDOW_NAME, cv2.CV_WINDOW_AUTOSIZE)
    cv2.cv.SetMouseCallback(WINDOW_NAME, manager.mouse_callback)

    rospy.init_node('map_manager')
    pub = rospy.Publisher('google_waypoints', Polygon, queue_size=10)
    r = rospy.Rate(10)
    polygon = Polygon()
    if rospy.is_shutdown():
        print 'borked'
    while not rospy.is_shutdown():
        points = manager.get_plotted_points_as_x_y_list()
        for i in range(len(points)):
            cv2.circle(manager.img, center=points[i], radius=5, color=RED, thickness=-1)
            if i > 0:
                cv2.line(manager.img, pt1=points[i-1], pt2=points[i], color=YELLOW, thickness = 2)
        cv2.imshow(WINDOW_NAME, manager.img)
        r.sleep()

        polygon.points = []
        for location in manager.plotted_points:
            point = Point32()
            point.x = location[0]
            point.y = location[1]
            point.z = 0

            polygon.points.append(point)

        pub.publish(polygon)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows
    cv2.waitKey(1)
    print manager.plotted_points
    #while not (cv2.waitKey(1) & 0xFF == ord('m')):
    #    pass
    #self.plotted_points = []

if __name__ == '__main__':
    start_map()