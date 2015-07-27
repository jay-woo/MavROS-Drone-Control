import serial
import math

def deg_min_to_deg_dec(raw): 
    '''
    convert coordinate from degrees, minutes to decimal degrees
    '''
    deg = 100*math.floor(raw/100)
    mins = raw-deg
    final = deg + (mins/0.6)
    return final

def get_GPS():
    '''
    outputs a single reading of latitue, longitude in decimal degrees
    '''
    ser = serial.Serial('/dev/ttyACM0', 9600)
    while True:
        line = ser.readline()
        data = line.split(',')
        if data[0] == '$GPRMC': #find the right line
            if data[2] == 'A': #check for good signal
                lon_raw = float(data[3]) #longitude in deg min
                lat_raw = float(data[5]) #latitute in deg min
                lon = deg_min_to_deg_dec(lon_raw)
                lat = deg_min_to_deg_dec(lat_raw)
                if data[4] == 'S': #make negative if needed
                    lon = -lon
                if data[6] == 'W':
                    lat = -lat
                return lat, lon
            else:
                return 'no signal'

if __name__ == '__main__':
    while True:
        print get_GPS()