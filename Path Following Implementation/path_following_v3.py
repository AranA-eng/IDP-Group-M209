from hardware import motor as mo
from hardware import sensors as sen
from control import PID, JunctionHandler
from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
import time
from utime import sleep

# motor pins
LEFT_PWM = 6
LEFT_DIR = 7
RIGHT_PWM = 5
RIGHT_DIR = 4
max_pwm = 65535

left_motor = mo.Motor(LEFT_DIR, LEFT_PWM)
right_motor = mo.Motor(RIGHT_DIR, RIGHT_PWM)



# line sensor pins
IR_PINS = [10, 12, 13, 11]
line_sensor_weights = [-5.0, -2.0, 2.0, 5.0]
sensors = sen.IRSensorArray( IR_PINS, line_sensor_weights)
left_junc_count = 0
right_junc_count = 0
cross_junc_count = 0


# Controller gain values
Kp = -2300.0
Ki = 20.0
Kd = 7.5

dt_ms = 10 # control loop period in ms

pid = PID(Kp, Ki, Kd, dt_ms, out_min= - max_pwm, out_max = max_pwm)







