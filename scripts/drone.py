class Drone():
    def __init__(self, drone_id):
        # Non-ROS variables
        self.drone_id = drone_id
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
       
        # ROS state variables
        self.armed = False
        self.flight_mode = 0
        self.voltage = 0
        self.current = 0
        self.battery_remaining = 0

    def state_callback(self, data):
        self.armed = data.armed
        self.flight_mode = data.mode

    def battery_callback(self, data):
        self.voltage = data.voltage
        self.current = data.current
        self.battery_remaining = data.remaining
