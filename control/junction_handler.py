from hardware import Motor
from utime import sleep




class JunctionHandler: # handling left and right motor
    """class for handling corners"""
    def __init__(self, motors, min_pwm, max_pwm):
        self.motors = motors  # motors must be instance of DifferentialDrive
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
        sleep(0.3)
        #---verify signs when we test---
        if dir == 1: #left turn
            self.motors.set(-55000, 55000)
            sleep(0.6)
        elif dir == -1: #right turn
            self.motors.set(55000, -55000)
            sleep(0.6)

        elif dir == 2:
            self.motors.set(-55000, 55000)
            sleep(1.3)

    

        

        

        

    
