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
i2c_colour = I2C(id = 1, sda = Pin(14), scl = Pin(15), freq = 400000)
i2c_VL53L0X = I2C(id = 0, sda = Pin(8), scl = Pin(9))                                               #Currently on the right side
i2c_TMF8701 = I2C(id = 0, sda = Pin(20), scl = Pin(21))                                             #Currently on the left side

# onoff pin for colour sensor                                                                      
onoff = Pin(14, Pin.OUT)
onoff.value(1)

# sensors initialisation
tof = DFRobot_TMF8701(i2c_TMF8701)
TMF8701 = sen.TMF8701(tof)                                                                          #now can write TMF8701.start(), .read(), .stop()
VL53L0X = sen.VL53L0X(i2c_VL53L0X)                                                                  #now can write VL53L0X.read(), .read(), .stop()
TCS3472 = sen.TCS34725(i2c_colour, onoffpin = onoff)                                                #now can write TCS3472.read()

VL53L0X.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
VL53L0X.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)

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
current_node = -1
count = 0
orange_counter = 0
purple_counter = 0
box_node = 0
ISON = None

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

def load(current_node, count):
    drive.stop()
    
    if current_node < 21:
        actuator.setheight(27)
    else:
        actuator.setheight(4)

    if IR_sensors.read() == [0, 1, 1, 1]:
        jh.turn(-1)  #turn right
    else:
        jh.turn(1)   #turn left
    
    while IR_sensors.read() != [0, 0, 0, 0]:     #setting motor speed to be very low and stop when the values read [0,0,0,0]
        vals = IR_sensors.read()
        err = IR_sensors.compute_error(vals)
        corr = pid.update(err)
        jh.apply_motor_speeds(10000, corr, 0)
        
    sleep(0.8)
    drive.stop()
    actuator.setheight(30)

    """                                                                                        #TODO: check this, would replace the two lines after   =====================================================
    timer = time.ticks_ms()																#timer to reverse
    while time.ticks_diff(time.ticks_ms(), timer) < 1800:
        vals = read_sensors()
        err = compute_error(vals)
        corr = -pid.update(err)
        apply_motor_speeds(-20000, corr, 0)
    """

    drive.set(-20000, -20000)                                                                  #reverse a bit out the shelf
    time.sleep(3)               

    
    drive.stop()
    minor_node = router.graph.get_node(current_node).minor.id    #set current node to current.minor
    jh.turn(2)                                                                                 #need to do a junc = 2 turn, then set count to 1
    count = 1    
    return minor_node, count

def unload(count):
    drive.set(10000, 10000)                                                                    #go a bit into the bay area
    time.sleep(2)
    drive.stop()
    actuator.setheight(0)
    drive.set(-20000, -20000)                                                                  #reverse a bit out the bay, simply needs to stop at the bay node
    time.sleep(2)                                                                              #ideally the robot will stop at the bay node since the sleep times are the same 
    drive.stop() 
    actuator.setheight(20)                                                                     #keep fork off ground in case we go to ramp
    jh.turn(2)                                                                                 #need to do a junc = 2 turn, then set count to 1
    count = 1
    return count

def i2c_clear(scl_pin, sda_pin):
    scl = Pin(scl_pin, Pin.OUT, value=1)
    sda = Pin(sda_pin, Pin.OUT, value=1)
    for _ in range(9):
        scl.low(); sleep(0.001)
        scl.high(); sleep(0.001)
    # STOP
    sda.low(); sleep(0.001)
    scl.high(); sleep(0.001)
    sda.high(); sleep(0.001)

def color_sensor_reading():
    clear, red, green, blue = TCS3472.read()

    if (abs(red - green) < (0.1 * clear)) and ((red - blue) > (0.05 * clear)): #yellow box: has lower blue and similar RG values
        color = "Yellow"
        #print("Yellow")
    elif (red > green) and (red > blue) and ((red-green) / clear > 0.05): #red box
        color = "Red"
        #print("Red")
    elif (blue > red) and (blue > green) and ((red-green) / clear > 0.05): #blue box
        color = "Blue"
        #print("Blue")
    else: #green box
        color = "Green"
        #print("Green")
    return color

def box_detection(vals, current_node, distance, threshold):    
    # the line sensor must read the junction values
    on_junction = vals in junction_sensor_values

    # the node at which the sensor just read must be in the bay areas
    in_bay_area = current_node in orange_branches or current_node in purple_branches
    
    # the distance sensor must be less than the threshold
    box_close = distance < threshold
    return on_junction and in_bay_area and box_close



threshold = 290                                                                                #measured this

color_node = {
"Green": 39,
"Blue": 40,
"Red": 41,
"Yellow": 42
}

color_node_val = [39,40,41,42]

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

#actuator.reset()
actuator.height = 0												#might need to change this based on bottom height of wooden forklift
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
            
            
            
            if current_node == 2 or current_node == 31:
                if ISON == "TMF":
                    TMF8701.end()
                device = "VL"
                i2c_clear(20, 21)
            
            elif current_node == 20 and current_node == 29:
                if ISON == "VL":
                    VL53L0X.end()
                device = "TMF"
                i2c_clear(8, 9)
            
            if device == "VL":
                distance = VL53L0X.read()
                ISON = "VL"
            elif device == "TMF":
                distance = TMF8701.read()
                ISON = "TMF"
                
                
            
            if box_detection(vals, current_node, distance, threshold):
                
                state = "COLLECTING"  
                current_node, count = load(current_node, count)                                       #need to traverse to minor: load returns the current node's minor
                color = color_sensor_reading()
                router.facing = "b"                                                                   #set facing to backwards (facing needs to be changed): this is required for the next turn sequence to start correctly 
                box_node = current_node

                if box_node in orange_branches: orange_counter += 1
                if box_node in purple_branches: purple_counter += 1

                # creating new route to deposit
                target_node = color_node[color]
                junction_list = router.route(box_node, target_node)
                turns = junction_list.turn_sequence
                nodes = junction_list.path_nodes

                state = "NAV_TO_DEPOSIT"
                
            if current_node == -1 and nodes[-1] == -1:                                                  #if reached node -1 and that is the end point (ie run is done) then stop
                sleep(1.5)
                drive.stop()
        
        elif state == "NAV_TO_DEPOSIT":
            jh.apply_motor_speeds(base_speed, corr, junction)
            if current_node in color_node_val:                                   
                state = "DEPOSIT"
        
        elif state == "DEPOSIT":
            count = unload(count)                                                                                     

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
                state = "START" # Done all 8                                                            #START state branch


            if next_area == "ORANGE":
                target_node = ORANGE_END_NODE
            elif next_area == "PURPLE":
                target_node = PURPLE_END_NODE
            
            else: 
                target_node = START_NODE

            junction_list = router.route(current_node, target_node)
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            count = 0                                                                                    #current reassigned based on the node sequence and the index of the node the box was found at: line 282
            
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
