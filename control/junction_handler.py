class JunctionHandler:
    def __init(self, motors, sensor_vals):
        self.motors = motors
        self.sensor_vals = sensor_vals

    def detect_junction(self, sensor_vals):
        if sensor_vals == [1,1,1,0]:
            return 1
        elif sensor_vals == [0,1,1,1]:
            return -1
        
        elif sensor_vals == [1,1,1,1]:
            return 1 # edit this bit please
        
        else: 
            return 0
        