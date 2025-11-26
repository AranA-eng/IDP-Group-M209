""" - Test for navigation. Sends the robot to every single bay.
    - Carries out box lift (any colour)
    - Deposits the box at node 40 
    MAIN GOALS
    - Test edge cases of robot_routing.py
    - Test the lift mechanism
    What is NOT being tested here?
    - colour sensors and distance TOF
    - Full game master logic
"""
"""
Test for navigation. Sends the robot to every single bay.
 - Collects a box (any colour)
 - Deposits the box at node 40

NOT TESTED:
 - colour sensor (TCS3472)
 - distance TOF
 - full game-state logic
"""

from hardware import motor as mo
from hardware import sensors as sen
from hardware import Actuator as act
from control import PID, JunctionHandler
from navigation import robot_routing as rt
from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
import time
from utime import sleep


# =========================
# Motor setup
# =========================

LEFT_PWM = 5
LEFT_DIR = 4
RIGHT_PWM = 6
RIGHT_DIR = 7
max_pwm = 65535

left_motor = mo.Motor(LEFT_DIR, LEFT_PWM)
right_motor = mo.Motor(RIGHT_DIR, RIGHT_PWM)
drive = mo.DiffDrive(left_motor, right_motor)


# =========================
# IRSensors + PID
# =========================

IR_PINS = [12, 13, 11, 10]
line_sensor_weights = [-5.0, -1.0, 1.0, 5.0]
IR_sensors = sen.IRSensorArray(IR_PINS, line_sensor_weights)

Kp = -2300.0
Ki = 20.0
Kd = 7.5
dt_ms = 10
base_speed = 45000

pid = PID(Kp, Ki, Kd, dt_ms, out_min=-max_pwm, out_max=max_pwm)
jh = JunctionHandler(drive, 0, max_pwm)


# ======================================================
# ROUTING GRAPH (nodedirections unchanged)
# ======================================================

nodedirections = {
    -1: (10, 10, 1, -1, 0),
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
    11: (10, 10, -1, 1, 30),
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

    # UPPER
    22: (10, 2, 10, 10, None),
    23: (10, 10, -1, 1, None),
    24: (10, 10, -1, 1, None),
    25: (10, 10, -1, 1, None),
    26: (10, 10, -1, 1, None),
    27: (10, 10, -1, 1, None),
    28: (10, 10, -1, 1, None),
    29: (1, -1, 10, 10, None),
    30: (10, 10, 1, -1, 11),
    31: (1, -1, 10, 10, None),
    32: (10, 10, -1, 1, None),
    33: (10, 10, -1, 1, None),
    34: (10, 10, -1, 1, None),
    35: (10, 10, -1, 1, None),
    36: (10, 10, -1, 1, None),
    37: (10, 10, -1, 1, None),
    38: (10, 2, 10, 10, None),

    # MINORS
    39: (10, 10, 1, -1, 1),
    40: (10, 10, 10, -1, 2),
    41: (10, 10, 1, 10, 20),
    42: (10, 10, 1, -1, 21),
}

graph = rt.RouteGraph(nodedirections)
router = rt.Router(graph)

actuator1 = act(dirPin=0, PWMPin=1)
actuator1.reset()


# ======================================
# Helpers
# ======================================

def get_minor(n):
    node = router.graph.get_node(n)
    return node.minor_id


def safe_minor(n):
    """Return minor node if it exists."""
    node = router.graph.get_node(n)
    return node.minor_id if node.minor_id not in (None, -1) else None


# Corrected BAY definitions:
ORANGE_LOWER = [3,4,5,6,7,8]
ORANGE_UPPER = [23,24,25,26,27,28]
PURPLE_LOWER = [12,13,14,15,16,17]      # <-- FIXED: these exist; 49–54 do not
PURPLE_UPPER = [32,33,34,35,36,37]

BAY_AREAS = ORANGE_LOWER + ORANGE_UPPER + PURPLE_LOWER + PURPLE_UPPER
BAY_AREAS_MINOR = [get_minor(n) for n in BAY_AREAS]


# ======================================
# LIFT MECHANISM (cleaned and correct)
# ======================================

def lift_mech(current_node, vals):
    """
    Move into bay → lift → return the minor node you end in.
    """
    node_obj = router.graph.get_node(current_node)

    # correct entry height
    actuator1.setheight(27 if current_node < 22 else 4)

    # move in slowly until inside bay
    if vals == [0,0,0,0]:
        drive.stop()
        time.sleep(0.2)

        # go to minor node (bay interior)
        minor_id = safe_minor(current_node)
        if minor_id is None:
            raise RuntimeError("Entered a bay node without a minor!")

        # raise slightly to grab the block
        actuator1.setheight(34)
        actuator1.stop()
        return minor_id

    return current_node


# ======================================
# TURN FOLLOWER
# ======================================

def turn_follower(junction_list, vals, count):
    """
    Detect junction and return (turn, next_index)
    """
    if vals in ([1,1,1,0], [0,1,1,1], [1,1,1,1]):
        return junction_list[count], count + 1
    return None


# ======================================
#     MAIN TEST LOOP
# ======================================

junction_list = router.route(-1, 3)      # start → first bay
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

        result = turn_follower(turns, vals, count)
        if result:
            turn, count = result
        else:
            turn = 0

        # -------------------------
        # STATE MACHINE
        # -------------------------

        if STATE == "FOLLOW LINE":
            if current_node == nodes[-2]:
                # entering bay
                jh.apply_motor_speeds(10000, corr, turn)
                bay_count += 1
                STATE = "COLLECTING"

                minor_node = lift_mech(current_node, vals)
                current_node = minor_node

                # plan route to deposit node 40
                junction_list = router.route(current_node, 40)
                turns = junction_list.turn_sequence
                nodes = junction_list.path_nodes
                count = 0

                STATE = "NAV TO DEPOSIT"

            else:
                jh.apply_motor_speeds(base_speed, corr, turn)

        elif STATE == "NAV TO DEPOSIT":
            jh.apply_motor_speeds(base_speed, corr, turn)

            if current_node == 40:
                STATE = "DEPOSIT"

        elif STATE == "DEPOSIT":
            actuator1.setheight(4)
            drive.set(-10000, -10000)
            sleep(0.3)
            drive.stop()

            # move to next bay
            target_minor = BAY_AREAS_MINOR[bay_count]
            junction_list = router.route(current_node, target_minor)
            turns = junction_list.turn_sequence
            nodes = junction_list.path_nodes
            count = 0

            STATE = "FOLLOW LINE"

        # ============================
        # Timing
        # ============================
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)

finally:
    drive.stop()
