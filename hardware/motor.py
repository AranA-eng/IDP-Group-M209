from machine import Pin, PWM

class Motor:
    def __init__(self, dir_pin, pwm_pin, max_pwm = 65535):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.max_pwm = max_pwm

    def set_speed(self, val):
        val = int(max(-self.max_pwm, min(self.max_pwm, val)))

        if val >= 0:
            self.dir.value(0)
            duty = val
        
        else: 
            self.dir.value(1)
            duty = -val
        
        self.pwm.duty_u16(duty)

class DiffDrive:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def set(self, left, right):
        self.left.set_speed(left)
        self.right.set_speed(right)

    def stop(self):
        self.left.set_speed(0)
        self.right.set_speed(0)




