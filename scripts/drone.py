class Drone():
    def __init__(self, drone_id):
        # Non-ROS variables
        self.drone_id = drone_id
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
       
        # ROS state variables
        self.armed = False
        self.flight_mode = 0
