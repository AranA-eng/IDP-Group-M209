from machine import Pin, ADC
from utime import sleep

MAX_RANGE = 520
ADC_SOLUTION = 3300.0
sen = ADC(Pin(26))

while True:
    adc_value = sen.read_u16()
    dist_t = adc_value * MAX_RANGE/ ADC_SOLUTION 
    print(dist_t)

    sleep(0.2)
