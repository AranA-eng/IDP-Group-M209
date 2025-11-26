from hardware import motor as mo
from hardware import sensors as sen
from hardware import Actuator as act
from control import PID, JunctionHandler
from navigation import robot_routing as rt
from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
import time
from utime import sleep

# motor pins
LEFT_PWM = 5
LEFT_DIR = 4
RIGHT_PWM = 6
RIGHT_DIR = 7
max_pwm = 65535

left_motor = mo.Motor(LEFT_DIR, LEFT_PWM)
right_motor = mo.Motor(RIGHT_DIR, RIGHT_PWM)



# line sensor pins
IR_PINS = [12, 13, 11, 10]
line_sensor_weights = [-5.0, -1.0, 1.0, 5.0]
IR_sensors = sen.IRSensorArray( IR_PINS, line_sensor_weights)

# Controller parameters
# Gain
Kp = -2300.0
Ki = 20.0
Kd = 7.5

dt_ms = 10 # control loop period in ms
base_speed = 45000 # base speed

pid = PID(Kp, Ki, Kd, dt_ms, out_min= - max_pwm, out_max = max_pwm)
drive = mo.DiffDrive(left_motor, right_motor)
jh = JunctionHandler(drive, 0, max_pwm)

nodedirections = {
    -1: (10, 10, 1, -1, 0), #note that for the minor nodes, minordirs means the direction to turn at their major links to go f or b
    0: (10, 10, 1, -1, -1),
    1: (10, 10, 1, -1, 39),
    2: (-1, 1, 1, 10, 40),
    3: (10, 10, -1, 1, None),
    4: (10, 10, -1, 1, None),
    5: (10, 10, -1, 1, None),
    6: (10, 10, -1, 1, None),
    7: (10, 10, -1, 1, None),
    8: (10, 10, -1, 1, None),
    9: (10, 10, 10, 10, None),
    10: (-1, 1, 10, 10, None),
    11: (10, 10, -1, 1, 30), #link to other main branch
    12: (-1, 1, 10, 10, None),
    13: (10, 10, 10, 10, None),
    14: (10, 10, -1, 1, None),
    15: (10, 10, -1, 1, None),
    16: (10, 10, -1, 1, None),
    17: (10, 10, -1, 1, None),
    18: (10, 10, -1, 1, None),
    19: (10, 10, -1, 1, None),
    20: (-1, 1, 10, -1, 41),
    21: (10, 10, 1, -1, 42),
    22: (10, 2, 10, 10, None),
    23: (10, 10, -1, 1, None),
    24: (10, 10, -1, 1, None),
    25: (10, 10, -1, 1, None),
    26: (10, 10, -1, 1, None),
    27: (10, 10, -1, 1, None),
    28: (10, 10, -1, 1, None),
    29: (1, -1, 10, 10, None),
    30: (10, 10, 1, -1, 11), #link to other main branch
    31: (1, -1, 10, 10, None),
    32: (10, 10, -1, 1, None),
    33: (10, 10, -1, 1, None),
    34: (10, 10, -1, 1, None),
    35: (10, 10, -1, 1, None),
    36: (10, 10, -1, 1, None),
    37: (10, 10, -1, 1, None),
    38: (10, 2, 10, 10, None),
    39: (10, 10, 1, -1, 1), #minor from 1
    40: (10, 10, 10, -1, 2), #minor from 2
    41: (10, 10, 1, 10, 20), #minor from 20
    42: (10, 10, 1, -1, 21), #minor from 21
}

graph = rt.RouteGraph(nodedirections) 
router = rt.Router(graph)

actuator1 = act(dirPin=0, PWMPin=1)
actuator1.reset()

    
def get_minor(n):
    return router.graph.get_node(n).minor_id

def lift_mech(current_node, vals):
    if current_node < 21:
        actuator1.setheight(27)
    else:
        actuator1.setheight(4)

    # setting motor speed to be very low and stop when the values read [0,0,0,0]
    if vals == [0,0,0,0]:
        time.sleep(0.2) # experiment w values here
        drive.stop()
        #set current node to current.minor

        minor_node = get_minor(current_node) # ------------------

    actuator1.setheight(34)
    actuator1.stop()
    return minor_node

def turn_follower(junction_list, vals, count):
    """ when sensor reads a junction, the junction identification is given by the corresponding counter
        returns junction value, and the next count """
    if vals == [1,1,1,0] or vals == [0,1,1,1] or vals ==[1,1,1,1]:
        junction = junction_list[count]
        next_count = count + 1 # next index in the junction_list
        return junction, next_count
    
def minor_list(list):
    return [get_minor(i) for i in list]

ORANGE_LOWER = [3,4,5,6,7,8]
ORANGE_UPPER = [23,24,25,26,27,28]
PURPLE_LOWER = [49, 50, 51, 52, 53, 54]
PURPLE_UPPER = [37, 36, 35, 34, 33, 32]

BAY_AREAS = ORANGE_LOWER + ORANGE_UPPER + PURPLE_LOWER + PURPLE_UPPER

ORANGE_LOW_MINOR = minor_list(ORANGE_LOWER)
ORANGE_UPPER_MINOR = minor_list(ORANGE_UPPER)
PURPLE_LOW_MINOR = minor_list(PURPLE_LOWER)
PURPLE_UPPER_MINOR = minor_list(PURPLE_UPPER)

BAY_AREAS_MINOR = minor_list(BAY_AREAS)

junction_list = router.route(-1, 43)
turns = junction_list.turn_sequence
nodes = junction_list.path_nodes
count = 0
bay_count = 0
STATE = "FOLLOW LINE"

try: 
    while True: 
        t_start = time.ticks_ms()


        vals = IR_sensors.read()
        err = IR_sensors.compute_error(vals)
        corr = pid.update(err)

        current_node = nodes[count]

        # once new node is detected, returns the behaviour that needs to happen, as well as incrementing count
        result = turn_follower(turns, vals, count)
        if result: 
            junction, count = result
        else: 
            junction = 0 # go straight

        # STATE MACHINE
        if STATE == "FOLLOW LINE":
            jh.apply_motor_speeds(base_speed, corr, junction)

            if current_node == nodes[-2]:
                jh.apply_motor_speeds(10000, corr, junction)
                bay_count += 1
                STATE = "COLLECTING"
                minor_node = lift_mech(current_node, vals)
                current_node = minor_node

                target_node = 40
                junction_list = router.route(current_node, target_node)
                turns = junction_list.turn_sequence
                nodes = junction_list.path_nodes
                count = 0

                STATE = "NAV TO DEPOSIT"

        elif STATE == "NAV TO DEPOSIT":
            jh.apply_motor_speeds(base_speed, corr, junction)
            if current_node == 40:
                STATE = "DEPOSIT"

        elif STATE == "DEPOSIT":
            
            # lowering block
            actuator1.setheight(4)

            # reverse slightly to remove fork from box
            drive.set(-10000, -10000)
            sleep(0.3) # experiment with this value
            drive.stop()

            target_node = BAY_AREAS_MINOR[bay_count]
            junction_list = router.route(current_node, target_node)
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            count = 0

            STATE = "FOLLOW LINE"

        
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)

finally:
    drive.stop()