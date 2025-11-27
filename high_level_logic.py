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

# led and button pins
led_pin = 27
led = Pin(led_pin, Pin.OUT)

button_pin = 28
button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

led_on = 0
prev = 0

# motor pins
LEFT_PWM = 5
LEFT_DIR = 4
RIGHT_PWM = 6
RIGHT_DIR = 7
max_pwm = 65535

# actuator pins
dirPin = 0
PWMPin = 1

left_motor = mo.Motor(LEFT_DIR, LEFT_PWM)
right_motor = mo.Motor(RIGHT_DIR, RIGHT_PWM)
actuator = act.Actuator(dirPin, PWMPin)

# i2c buses
i2c_colour = I2C(id = 1, sda = Pin(14), scl = Pin(15), freq = 400000)                           #CHECK: check these
i2c_VL53L0X = I2C(id = 0, sda = Pin(2), scl = Pin(3))                                           #CHECK: on the left?
i2c_TMF8701 = I2C(id = 1, sda = Pin(8), scl = Pin(9))                                           #CHECK: on the right?

# onoff pin for colour sensor                                                                   #CHECK: check pin value
onoff = Pin(14, Pin.out)
onoff.value(1)

# sensors initialisation
tof = DFRobot_TMF8701(i2c_TMF8701)
TMF8701 = TMF8701(tof)                                                                          #DONE: now can write TMF8701.start(), .read(), .stop()
VL53L0X = VL53L0X(i2c_VL53L0X)                                                                  #DONE: now can write VL53L0X.read()
TCS3472 = TCS34725(i2c_colour, onoffpin = onoff)                                                #DONE: now can write TCS3472.read()

# line sensor pins
IR_PINS = [12, 13, 11, 10]
line_sensor_weights = [-5.0, -2.0, 2.0, 5.0]
IR_sensors = sen.IRSensorArray(IR_PINS, line_sensor_weights)

# Controller parameters
# Gain
Kp = 2300.0
Ki = -20.0
Kd = -7.5

dt_ms = 10 # control loop period in ms
base_speed = 45000 # base speed

pid = PID(Kp, Ki, Kd, dt_ms, out_min= - max_pwm, out_max = max_pwm)
drive = mo.DiffDrive(left_motor, right_motor)
jh = JunctionHandler(drive, 0, max_pwm)
count = 0
orange_counter = 0
purple_counter = 0
box_node = 0

orange_branches = [3, 4, 5, 6, 7, 8, 23, 24, 25, 26, 27, 28]
purple_branches = [14, 15, 16, 17, 18, 19, 32, 33, 34, 35, 36, 37]

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





def turn_follower(junction_list, vals, count):
    """ when sensor reads a junction, the junction identification is given by the corresponding counter
        returns junction value, and the next count """
    if vals == [1,1,1,0] or vals == [0,1,1,1] or vals ==[1,1,1,1]:
        junction = junction_list[count]
        next_count = count + 1 # next index in the junction_list
        return junction, next_count

def load(current_node):
    drive.stop()
    
    if current_node < 21:
        actuator.setheight(27)
    else:
        actuator.setheight(4)

    if IR_sensors.read() == [0, 1, 1, 1]:
        #turn right																						
    else:
        #turn left																						
    
    while IR_sensors.read() != [0, 0, 0, 0]:     #setting motor speed to be very low and stop when the values read [0,0,0,0]
        vals = IR_sensors.read()
        err = IR_sensors.compute_error(vals)
        corr = pid.update(err)
        jh.apply_motor_speeds(10000, corr, 0)
    sleep(0.2)
    drive.stop()
    actuator.setheight(34)
    drive.set(-20000, -20000)                                                                  #DONE: reverse a bit out the shelf
    time.sleep(3)                                                                              #TODO: need to enforce first turn of turn_sequence and set counter = 1
    drive.stop()
    minor_node = router.graph.get_node(current_node).minor.id    #set current node to current.minor
    .													#need to do a junc = 2 turn, then set count to 1 I THINK
    return minor_node

def unload():
    drive.set(10000, 10000)                                                                    #DONE: go a bit into the bay area
    time.sleep(2)
    drive.stop()
    actuator.setheight(0)
    drive.set(-20000, -20000)                                                                  #DONE: reverse a bit out the bay, simply needs to stop at the bay node
    time.sleep(1.3)                                                                            #DONE: ideally the robot will stop at the bay node since the sleep times are the same 
    drive.stop()                                                                               #QUESTION: if the robot does this accurately for 2 boxes of the same colour, will it try to stack one directly on top of the other (fork will snap)?
    actuator.setheight(20) #keep fork off ground in case we go to ramp
    .													#need to do a junc = 2 turn, then set count to 1 I THINK
    return

def color_sensor_reading():
    color_reading = TCS3472.read()
    
    if color_reading > 2000:                                                                    #CHECK: need boundaries
        color = "Green"
    elif color_reading > 1500:
        color = "Red"
    elif color_reading > 1000:
        color = "Yellow"
    else:
        color = "Blue"
    return color

def distance_sensor_reading():
    if IR_sensors.read() == [0, 1, 1, 1]:      #need to use right sensor
        distance = 															#which sensor on which side?
    elif IR_sensors.read() == [1, 1, 1, 0]:    #need to use left sensor
        distance = 															#which sensor on which side?
    else:
        distance = 0  #doesn't affect logic for box_detection since this case won't be while a junction is detected and in the correct node space
    return distance

def box_detection(vals, current_node, distance, threshold):
    # the line sensor must read the junction values
    on_junction = vals in junction_sensor_values

    # the node at which the sensor just read must be in the bay areas
    in_bay_area = current_node in orange_branches or current_node in purple_branches
    
    # the distance sensor must be less than the threshold
    box_close = distance < threshold
    return on_junction and in_bay_area and box_close



threshold = 200                                                                                #CHECK: measure this

color_node = {
"Green": 1,
"Blue": 2,
"Red": 20,
"Yellow": 21
}

color_node_val = [1,2,20,21]

ORANGE_END_NODE = 22
PURPLE_END_NODE = 38
START_NODE = 0

state = "FOLLOW_LINE"

cold = ["Blue", "Green"]
warm = ["Red", "Yellow"]

junction_sensor_values = [[0,1,1,1], [1,1,1,0], [1,1,1,1]]

graph = rt.RouteGraph(nodedirections) 
router = rt.Router(graph)
junction_list = router.route(0, 22)
turns = junction_list.turn_sequence
nodes = junction_list.path_nodes



try:
    while True:
        button_val = button.value()
        if button_val == 1 and prev == 0:
            break
finally:
    pass

led.value(1)

actuator.reset()
actuator.setheight(20)

try: 
    while True: 
        t_start = time.ticks_ms()

        vals = IR_sensors.read()
        err = IR_sensors.compute_error(vals)
        corr = pid.update(err)

        # CURRENT JUNCTION
        current_node = nodes[count]
        
        # once new node is detected, returns the behaviour that needs to happen, as well as incrementing count
        result = turn_follower(turns, vals, count)
        
        if result is not None: 
            junction, count = result
        else: 
            junction = 0 # go straight

        # STATE MACHINE
        if state == "FOLLOW_LINE":
            actuator.setheight(20)
            jh.apply_motor_speeds(base_speed, corr, junction)

            distance = distance_sensor.read()                                                         #TODO: which distance sensor you use depends on which shelves you are at and which way round the arena you are travelling
            if box_detection(vals, current_node, distance, threshold):                                #eg: if current_node in purple branches, if current_node < 21, then read from left sensor, maybe need to know next node? ie from next index
                
                state = "COLLECTING"  
                current_node = load(current_node)                                                     #DONE:need to traverse to minor: load returns the current node's minor
                color = color_sensor_reading()
                router.facing = "b"                                                                   #DONE: set facing to backwards (one of two cases where facing needs to be changed): this is required for the next turn sequence to start correctly 
                box_node = current_node

                if box_node in orange_branches: orange_counter += 1
                if box_node in purple_branches: purple_counter += 1

                # creating new route to deposit
                target_node = color_node[color]
                junction_list = router.route(box_node, target_node)
                turns = junction_list.turn_sequence
                nodes = junction_list.path_nodes
                count = 0 # reset counter

                state = "NAV_TO_DEPOSIT"
                
            if current_node == -1 and nodes[-1] == -1:                                                  #DONE: if reached node -1 and that is the end point (ie run is done) then stop
                sleep(1.5)
                drive.stop()
        
        elif state == "NAV_TO_DEPOSIT":
            jh.apply_motor_speeds(base_speed, corr, junction)
            if current_node in color_node_val:                                   
                state = "DEPOSIT"
        
        elif state == "DEPOSIT":
            unload()                                                                                     

            if orange_counter < 4 and purple_counter < 4: 
                # both areas have boxes left
                if color in cold: 
                    next_area = "ORANGE"
                
                if color in warm: 
                    next_area = "PURPLE"
                
            elif orange_counter < 4:
                next_area = "ORANGE"

            elif purple_counter < 4:
                next_area = "PURPLE"

            else: 
                state = "START" # Done all 8                                                            #DONE: START state branch


            if next_area == "ORANGE":
                target_node = ORANGE_END_NODE
            elif next_area == "PURPLE":
                target_node = PURPLE_END_NODE
            
            else: 
                target_node = START_NODE

            junction_list = router.route(current_node, target_node)
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            count = 0                                                                                    #DONE: current reassigned based on the node sequence and the index of the node the box was found at: line 282
            
            state = "FOLLOW_LINE"
                
        elif state == "START":
            junction_list = router.route(current_node, -1)
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            state = "FOLLOW_LINE"
        
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)

finally:
    drive.stop()
    
    
#can use interrupts: index error: continue
    #is that even needed? maybe because we keep trying to work towards 22 and 38 then we never reach an index error?
    
    
#TODOS:
#How are we implementing turn now? given that it now uses the IR sensor readings, are we now going to import the IR sensors into the control lib?
#Enforcing these turns after the load/unload sequences (this is entirely dependent on the previous point)
#Which distance sensor on which side?
