from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
import time
from utime import sleep

IR_PINS = [10, 12, 13, 11] #adjust accordingly

# Motor pins - adjust accordingly -------------------------------------------------------------------------------------------------------
motor_left_pwm_pin = 6
motor_left_dir_pin = 7
motor_right_pwm_pin = 5
motor_right_dir_pin = 4


#---Parameters---
weights = [-5.0, -1.0, 1.0, 5.0] # position from the center of each sensor
base_speed = 50000 # pwm duty
max_pwm = 65535
min_pwm = 15000

# how were these values chosen?
Kp = -1500.0
Ki = 0.0
Kd = -0.05

dt_ms = 10 # control loop period in ms

# --- sensor setup
# Include sensor setup here plz ----------------------------------------------------------------------------------------
sensors = [Pin(IR, Pin.IN) for IR in IR_PINS]




#--- Motor Class
class Motor:
    def __init__(self, dir_pin, PWMPin):
        self.mDir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin)) # set motor pwm pin
        self.pwm.freq(1000) # set PWM frequeuncy
        self.pwm.duty_u16(0) # set duty cycle - 0 = off
    
    def set(self, speed):
        if speed >= 0:
            self.mDir.value(0) # forward is 0, like in the test code
            duty = min(int(speed), max_pwm) # saturates the wheel speed
        else:
            self.mDir.value(1)
            duty = min(int(-speed), max_pwm)
        self.pwm.duty_u16(duty)

left_motor = Motor(motor_left_dir_pin, motor_left_pwm_pin)
right_motor = Motor(motor_right_dir_pin, motor_right_pwm_pin)

#--- PID Class ---
class PID:
    def __init__(self, kp, ki, kd, dt_ms, out_min = -1e9, out_max = 1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt_ms / 1000
        self.integral = 0.0
        self.prev_err = 0.0
        self.out_min, self.out_max = out_min, out_max
        
    
    def update(self, err):
        self.integral += err*self.dt
         
        deriv = (err - self.prev_err) / self.dt if self.dt > 0 else 0.0
        
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        
        if out > self.out_max:
            out = self.out_max
            
        elif out < self.out_min:
            out = self.out_min
        self.prev_err = err
        return out
    
    
pid = PID(Kp, Ki, Kd, dt_ms, out_min=-base_speed, out_max=base_speed)

def read_sensors():
    sleep(0.3)
    return [s.value() for s in sensors]


# --- function that computes the error---
def compute_error(vals):
    total = 0.0
    weight_sum = 0.0
    for w, v in zip(weights, vals):
        total += w * v
        weight_sum += v
    if weight_sum == 0:
        return 0
    return total / weight_sum


# beware of signs here for left and right
def apply_motor_speeds(base, correction, junction):
    junction_boost = 55000  # arbitrary increase in PWM for turning
    
    left = base + correction + junction * junction_boost
    right = base - correction - junction * junction_boost

    left = max(min_pwm, min(max_pwm, int(left)))
    right = max(min_pwm, min(max_pwm, int(right)))
    
    if junction == 0:
        left_motor.set(left)
        right_motor.set(right)
    else:
        left_motor.set(left)
        right_motor.set(right)        
        sleep(0.8)
    
def stop_all():
    left_motor.set(0)
    right_motor.set(0)
    
try:
    while True:
        t_start = time.ticks_ms()
        vals = read_sensors()
        err = compute_error(vals)
        corr = pid.update(err)
        
        if vals == [1, 1, 1, 0]:
            junction = 1
        elif vals == [0, 1, 1, 1]:
            junction = -1
        elif vals == [1, 1, 1, 1]:
            junction = 1
        else:
            junction = 0
        
        apply_motor_speeds(base_speed, corr, junction)
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)
            


finally:
    stop_all()

