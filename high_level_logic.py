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

# motor pins
LEFT_PWM = 6
LEFT_DIR = 7
RIGHT_PWM = 5
RIGHT_DIR = 4
max_pwm = 65535

# actuator pins
dirPin = 0
PWMPin = 1

left_motor = mo.Motor(LEFT_DIR, LEFT_PWM)
right_motor = mo.Motor(RIGHT_DIR, RIGHT_PWM)
actuator = act.Actuator(dirPin, PWMPin)

# i2c buses																											
i2c_colour = I2C(id = 1, sda = Pin(14), scl = Pin(15), freq = 400000)           #check these
i2c_VL53L0X = I2C(id = 0, sda = Pin(2), scl = Pin(3))                         #on the left?
i2c_TMF8701 = I2C(id = 1, sda = Pin(8), scl = Pin(9))                         #on the right?

# onoff pin for colour sensor                                                 #check pin value
onoff = Pin(14, Pin.out)									
onoff.value(1)

# sensors initialisation
tof = DFRobot_TMF8701(i2c_TMF8701)									
TMF8701 = TMF8701(tof) 										                    #now can write TMF8701.start(), .read(), .stop()
VL53L0X = VL53L0X(i2c_VL53L0X)								                    #now can write VL53L0X.read()
TCS3472 = TCS34725(i2c_colour, onoffpin = onoff)								#now can write TCS3472.read()

# line sensor pins
IR_PINS = [10, 12, 13, 11]
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





def turn_follower(junction_list, vals, count):
    """ when sensor reads a junction, the junction identification is given by the corresponding counter
        returns junction value, and the next count """
    if vals == [1,1,1,0] or vals == [0,1,1,1] or vals ==[1,1,1,1]:
        junction = junction_list[count]
        next_count = count + 1 # next index in the junction_list
        return junction, next_count

def lift_mech(router, ):                                                          #need to put DiffDrive or motors in this
        current = router.current
        #needs to turn be junction == current.minorfdir or current.minorbdir depending on facing at that node
        if current.data < 21: #on lower level
            actuator.setheight(27)
        else: #on upper level
            actuator.setheight(4)  
        
        #set motor speeds to very low and stop when vals read [0,0,0,0]
        
        actuator.setheight(34)
        actuator.stop()
        
        #reverse, 180 turn, forward until [1,1,1,1], turn at junction by (-1)(current.minorfdir if facing == "f" else current.minorbdir)

def lift_mech(current_node, vals):
    if current_node < 21:
        actuator.setheight(27)
    else:
        actuator.setheight(4)

    # setting motor speed to be very low and stop when the values read [0,0,0,0]
    drive.set(10000,10000)
    if vals == [0,0,0,0]:
        time.sleep(0.2) # experiment w values here
        drive.stop()
        #set current node to current.minor

        minor_node = router.graph.get_node(current_node).minor # ------------------

    actuator.setheight(34)
    actuator.stop()
    return minor_node

    

def unload():
    actuator.setheight(0)
    #reverse a little bit
    actuator.setheight(20) #keep fork off ground in case we go to ramp
    actuator.stop()
    return

def color_sensor_reading():
    return None

threshold = 100

color_node = {
"green": 1,
"blue": 2,
"red": 20,
"yellow": 21
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

def box_detection(vals, current_node, distance, threshold):
    # the line sensor must read the junction values
    on_junction = vals in junction_sensor_values

    # the node at which the sensor just read must be in the bay areas
    in_bay_area = current_node in orange_branches or current_node in purple_branches
    
    # the distance sensor must be less than the threshold
    box_close = distance < threshold
    return on_junction and in_bay_area and box_close

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
        if result: 
            junction, count = result
        else: 
            junction = 0 # go straight

        # STATE MACHINE
        if state == "FOLLOW_LINE":
            actuator.setheight(20)
            jh.apply_motor_speeds(base_speed, corr, junction)

            distance = distance_sensor.read() #-------------------------------------------------------------------------
            if box_detection(vals, current_node, distance, threshold):
                
                state = "COLLECTING"
                lift_mech()
                color = color_sensor_reading()

                box_node = current_node #current_node.minor

                if box_node in orange_branches: orange_counter += 1
                if box_node in purple_branches: purple_counter += 1

                # creating new route to deposit
                target_node = color_node[color]
                junction_list = router.route(box_node, target_node)
                turns = junction_list.turn_sequence
                nodes = junction_list.path_nodes
                count = 0 # reset counter

                state = "NAV_TO_DEPOSIT"
        
        elif state == "NAV_TO_DEPOSIT":
            jh.apply_motor_speeds(base_speed, corr, junction)
            if current_node in color_node_val:                                              #
                state = "DEPOSIT"
        
        elif state == "DEPOSIT":
            deposit_mechanism()                                                     #being written

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
                state = "START" # Done all 8


            if next_area == "ORANGE":
                target_node = ORANGE_END_NODE
            elif next_area == "PURPLE":
                target_node = PURPLE_END_NODE
            
            else: 
                target_node = START_NODE

            junction_list = router.route(current_node, target_node)                #is this meant to be router.route(current_node(), target_node)?
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            count = 0                                                                     #also, current needs to be reassigned based on when the interrupt was. we know the node sequence and the index of the node the box was found at
            
            state = "FOLLOW_LINE"
        

        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)

finally:
    mo.DiffDrive.stop()




