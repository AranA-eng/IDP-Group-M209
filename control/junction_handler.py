from hardware import Motor
from hardware import DifferentialDrive as DD
from utime import sleep




class JunctionHandler: # handling left and right motor
    def __init(self, motors, sensor_vals, base, correction, junction, min_pwm, max_pwm, leftmotor, rightmotor):
        self.motors = motors
        self.sensor_vals = sensor_vals
        self.base = base
        self.correction = correction
        self.junction = junction
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.leftmotor = leftmotor
        self.rightmotor = rightmotor

    def apply_motor_speeds(base, correction, junction, min_pwm, max_pwm):
        left = base + correction
        right = base - correction

        left = max(min_pwm, min(max_pwm, int(left)))
        right = max(min_pwm, min(max_pwm, int(right)))

        # DD.set(left, right)

        if junction == 0:
            sleep(0.03)
        
        elif junction == 10:
            sleep(0.4)

        elif junction == 1:
            turn(junction)

        elif junction == -1:
            turn(junction)

    def turn(dir):
        if dir == 1:
            sleep(0.45)
            DD.set(-55000, 55000)
            sleep(0.6)
        elif dir == -1:
            sleep(0.45)
            DD.set(-55000, 55000)
            sleep(0.6)

        elif dir == 2:
            sleep(0.45)
            DD.set(-55000, 55000)
            sleep(1.3)

    

        

        

        

    