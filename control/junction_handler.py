from hardware import Motor, IRSensorArray
from utime import sleep




class JunctionHandler: # handling left and right motor
    """class for handling corners"""
    def __init__(self, motors, min_pwm, max_pwm, sensors):
        self.motors = motors  # motors must be instance of DifferentialDrive
        self.sensors = sensors  # sensors must be instance of IRSensorArray
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
    
    def detect_junction(self, vals):
        if vals == [1,1,1,0]: return 'LEFT'
        if vals == [0,1,1,1]: return 'RIGHT'
        if vals == [1,1,1,1]: return 'CROSS'
        return None

    def apply_motor_speeds(self, base, correction, junction):
        left = base + correction
        right = base - correction

        left = max(self.min_pwm, min(self.max_pwm, int(left)))
        right = max(self.min_pwm, min(self.max_pwm, int(right)))

        self.motors.set(left, right)

        if junction == 0:
            pass
            #sleep(0.03)
        
        elif junction == 10:
            sleep(0.25)

        elif junction == 1 or junction == -1:
            self.turn(junction)

    def turn(self, dir):
        sleep(0.4)
        #---verify signs when we test---    dir == 1 means left turn
        while read_sensors() != [0, 1, 1, 0]:
            #print(read_sensors())
            left_motor.set(-55000)
            right_motor.set(55000)

            
        elif dir == -1: #right turn
            sleep(0.4)
            while read_sensors() != [0, 1, 1, 0]:
                left_motor.set(55000)
                right_motor.set(-55000)
    
    
        elif dir == 2:
            sleep(0.2)
            left_motor.set(-55000)
            right_motor.set(55000)
            sleep(0.2)
            while read_sensors() != [0, 1, 1, 0]:
                left_motor.set(-55000)
                right_motor.set(55000)

    

        

        

        

    
