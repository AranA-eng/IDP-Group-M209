from hardware import motor as mo
from hardware import sensors as sen
from control import PID, JunctionHandler
from navigation import turn_counter
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

# --- Junction decisions for lap test ---
right_juncs = {
#assuming we turn left at the beginning, going round CW
1: 10, #ignore: bay6 orange
2: 10, #ignore: bay5 orange
3: 10, #ignore: bay4 orange
4: 10, #ignore: bay3 orange
5: 10, #ignore: bay2 orange
6: 10, #ignore: bay1 orange
7: -1, #turn right: major corner
8: 10, #ignore: ramp
9: -1, #turn right: major corner 
10: 10, #ignore: bay1 purple
11: 10, #ignore: bay2 purple
12: 10, #ignore: bay3 purple
13: 10, #ignore: bay4 purple
14: 10, #ignore: bay5 purple
15: 10, #ignore: bay6 purple
16: -1, #turn right: final major turn
17: 10 #ignore: buffer
}

left_juncs = {
#assuming we turn left at the beginning, going round CW
1: 10, #ignore: green bay 
2: 10, #ignore: yellow bay
3: 1, #turn left: ending
4: 10 #ignore: buffer
}

cross_juncs = {
1: 10, #ignore: inside starting box
2: 1, #turn left: beginning
3: -1, #turn right: major corner
4: 10, #ignore: random +
5: 10, #ignore: random +
6: 10 #ignore: inside starting box
}


# Controller parameters
# Gain
Kp = 2300.0
Ki = -20.0
Kd = -7.5

dt_ms = 10 # control loop period in ms
base_speed = 45000 # base speed

pid = PID(Kp, Ki, Kd, dt_ms, out_min= - max_pwm, out_max = max_pwm)
tc = turn_counter()
drive = mo.DiffDrive(left_motor, right_motor)
jh = JunctionHandler(drive, 0, max_pwm)

try: 
    while True: 
        t_start = time.ticks_ms()
        vals = IR_sensors.read()
        err = IR_sensors.compute_error(vals)
        corr = pid.update(err)
        direction = jh.detect_junction(vals)

        junction = tc.direction(vals, left_juncs, right_juncs, cross_juncs)

        if tc.cross_junc_count == 6 or tc.left_junc_count == 4 or tc.right_junc_count == 17:
            sleep(1.5)
            drive.stop()
            break

        jh.apply_motor_speeds(base_speed, corr, junction)
        
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < dt_ms:
            time.sleep_ms(dt_ms - elapsed)

finally:
    drive.stop()
