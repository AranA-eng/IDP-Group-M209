from hardware import motor as mo
from hardware import sensors as sen
from hardware import linear_actuator as act
from control import PID, JunctionHandler
from navigation import robot_routing as rt
from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701
import time
from utime import sleep

IR_PINS = [13, 12, 11, 10] #adjust accordingly

# Motor pins - adjust accordingly -------------------------------------------------------------------------------------------------------
motor_left_pwm_pin = 5
motor_left_dir_pin = 4
motor_right_pwm_pin = 6
motor_right_dir_pin = 7


#---Parameters---
weights = [-5.0, -1.0, 1.0, 5.0] # position from the center of each sensor
base_speed = 50000 # pwm duty
max_pwm = 65535
min_pwm = 15000

# how were these values chosen?
Kp = 2300.0
Ki = -20.0
Kd = -7.5

dt_ms = 10 # control loop period in ms

# --- sensor setup
# Include sensor setup here plz ----------------------------------------------------------------------------------------
sensors = [Pin(IR, Pin.IN) for IR in IR_PINS]


led_pin = 27
led = Pin(led_pin, Pin.OUT)

#Set the button pin
button_pin = 28
button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

#Continuously update the LED value and print said value

led_on = 0
prev = 0


# map of the nodes
nodedirections = {
    -1: (10, 10, 1, -1, 0), #note that for the minor nodes, minordirs means the direction to turn at their major links to go f or b
    0: (10, 10, 1, -1, -1),
    1: (10, 10, 1, -1, 39),
    2: (-1, 1, 1, 10, 40),
    3: (10, 10, -1, 1, 43),
    4: (10, 10, -1, 1, 44),
    5: (10, 10, -1, 1, 45),
    6: (10, 10, -1, 1, 46),
    7: (10, 10, -1, 1, 47),
    8: (10, 10, -1, 1, 48),
    9: (10, 10, 10, 10, None),
    10: (-1, 1, 10, 10, None),
    11: (10, 10, -1, 1, 30), #link to other main branch
    12: (-1, 1, 10, 10, None),
    13: (10, 10, 10, 10, None),
    14: (10, 10, -1, 1, 49),
    15: (10, 10, -1, 1, 50),
    16: (10, 10, -1, 1, 51),
    17: (10, 10, -1, 1, 52),
    18: (10, 10, -1, 1, 53),
    19: (10, 10, -1, 1, 54),
    20: (-1, 1, 10, -1, 41),
    21: (10, 10, 1, -1, 42),
    22: (10, 2, 10, 10, None),
    23: (10, 10, -1, 1, 55),
    24: (10, 10, -1, 1, 56),
    25: (10, 10, -1, 1, 57),
    26: (10, 10, -1, 1, 58),
    27: (10, 10, -1, 1, 59),
    28: (10, 10, -1, 1, 60),
    29: (1, -1, 10, 10, None),
    30: (10, 10, 1, -1, 11), #link to other main branch
    31: (1, -1, 10, 10, None),
    32: (10, 10, -1, 1, 61),
    33: (10, 10, -1, 1, 62),
    34: (10, 10, -1, 1, 63),
    35: (10, 10, -1, 1, 64),
    36: (10, 10, -1, 1, 65),
    37: (10, 10, -1, 1, 66),
    38: (10, 2, 10, 10, None),
    39: (10, 10, 1, -1, 1), #minor from 1
    40: (10, 10, 10, -1, 2), #minor from 2
    41: (10, 10, 1, 10, 20), #minor from 20
    42: (10, 10, 1, -1, 21), #minor from 21

    #shelf nodes
    43: (10, 10, -1, 1, 3),
    44: (10, 10, -1, 1, 4),
    45: (10, 10, -1, 1, 5),
    46: (10, 10, -1, 1, 6),
    47: (10, 10, -1, 1, 7),
    48: (10, 10, -1, 1, 8),
    
    49: (10, 10, -1, 1, 14),
    50: (10, 10, -1, 1, 15),
    51: (10, 10, -1, 1, 16),
    52: (10, 10, -1, 1, 17),
    53: (10, 10, -1, 1, 18),
    54: (10, 10, -1, 1, 19),
    
    55: (10, 10, -1, 1, 23),
    56: (10, 10, -1, 1, 24),
    57: (10, 10, -1, 1, 25),
    58: (10, 10, -1, 1, 26),
    59: (10, 10, -1, 1, 27),
    60: (10, 10, -1, 1, 28),
    
    61: (10, 10, -1, 1, 32),
    62: (10, 10, -1, 1, 33),
    63: (10, 10, -1, 1, 34),
    64: (10, 10, -1, 1, 35),
    65: (10, 10, -1, 1, 36),
    66: (10, 10, -1, 1, 37)
}


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
    sleep(0.05)
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
    left = base + correction
    right = base - correction

    left = max(min_pwm, min(max_pwm, int(left)))
    right = max(min_pwm, min(max_pwm, int(right)))

    left_motor.set(left)
    right_motor.set(right)

    if junction == 0:
        sleep(0.03)
    
    elif junction == 10:
        sleep(0.4)

    elif junction == 1 or junction == -1:
        turn(junction)
    
def turn(dir):
    #---verify signs when we test---
    if dir == 1: #left turn
        sleep(0.4)
        left_motor.set(-55000)
        right_motor.set(55000)
        sleep(0.38)
    elif dir == -1: #right turn
        sleep(0.4)
        left_motor.set(55000)
        right_motor.set(-55000)
        sleep(0.38)

    elif dir == 2:
        sleep(0.4)
        left_motor.set(-55000)
        right_motor.set(55000)
        sleep(0.76)
    
def stop_all():
    left_motor.set(0)
    right_motor.set(0)
    
    
def turn_follower(junction_list, vals, count):
    """ when sensor reads a junction, the junction identification is given by the corresponding counter
        returns junction value, and the next count """
    if vals == [1,1,1,0] or vals == [0,1,1,1] or vals ==[1,1,1,1]:
        junction = junction_list[count]
        next_count = count + 1 # next index in the junction_list
        return junction, next_count
    
junction_sensor_values = [[0,1,1,1], [1,1,1,0], [1,1,1,1]]

graph = rt.RouteGraph(nodedirections) 
router = rt.Router(graph)
junction_list = router.route(-1, 22)
turns = junction_list.turn_sequence
nodes = junction_list.path_nodes
count = 0
print(turns)
print(nodes)

try:
    while True:
        button_val = button.value()
        if button_val == 1 and prev == 0:
            break
finally:
    pass


try:
    while True:
        print("starting")
        led_on = (led_on + 1) % 2
        led.value(led_on)
        sleep(0.25)
        
        t_start = time.ticks_ms()
        vals = read_sensors()
        err = compute_error(vals)
        corr = pid.update(err)
        
        current_node = nodes[count]
        
        # once new node is detected, returns the behaviour that needs to happen, as well as incrementing count
        result = turn_follower(turns, vals, count)
        if result is not None:																						# 
            junction, count = result
            print(junction)
        else: 
            junction = 0 # go straight
        print(f"applying {junction} to motors")
        apply_motor_speeds(base_speed, corr, junction)
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)
            


finally:
    stop_all()







