import xml.etree.ElementTree as ET
import rospkg
from mavros.msg import Waypoint

def get_mission(mission_num=0):
    rospack = rospkg.RosPack() 
    package_path = rospack.get_path('drone_control')
    tree = ET.parse(package_path + '/scripts/missions.xml')

    for mission in tree.getroot():
        if int(mission.attrib['id']) == mission_num:
            break

    waypoint_list = []
    for item in mission:
        waypoint = Waypoint()

        waypoint.frame = int(item.find('frame').text)
        waypoint.command = int(item.find('command').text)
        waypoint.is_current = bool(item.find('is_current').text)
        waypoint.autocontinue = bool(item.find('autocontinue').text)

        waypoint.param1 = float(item.find('param1').text)
        waypoint.param2 = float(item.find('param2').text)
        waypoint.param3 = float(item.find('param3').text)
        waypoint.param4 = float(item.find('param4').text)
        waypoint.x_lat = float(item.find('latitude').text)
        waypoint.y_long = float(item.find('longitude').text)
        waypoint.z_alt = float(item.find('altitude').text)
        
        waypoint_list.append(waypoint)

    return waypoint_list

def make_global_waypoint(lat, lon):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drone_control')
    waypoint = Waypoint()

    waypoint.frame = 3
    waypoint.command = 16
    waypoint.is_current = 0
    waypoint.autocontinue = False
    waypoint.param1 = 5 #hold time
    waypoint.param2 = 2
    waypoint.param3 = 0
    waypoint.param4 = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = 5

    return waypoint
