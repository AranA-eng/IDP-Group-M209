from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.VL53L0X.VL53L0X import VL53L0X
import time


class IRSensorArray: 
    def __init__(self, pins, weights):
        self.sensors = [Pin(p, Pin.IN) for p in pins]
        self.weights = weights

    def read(self):
        time.sleep_ms(7.5)
        return [s.value() for s in self.sensors]
    
    def compute_error(self, vals):
        if vals in ([1,1,1,0], [0,1,1,1], [1,1,1,1]):
            return 0
        
        total, weight_sum = 0,0
        for w,v in zip(self.weights, vals):
            total += w * v
            weight_sum += v
        return 0 if weight_sum == 0 else total/weight_sum


class TOFSensor:
    def __init__(self, i2c_id, sda_pin, scl_pin):
        self.i2c = I2C(id = i2c_id, sda = Pin(sda_pin), scl = Pin(scl_pin))
        self.sensor = VL53L0X(i2c)
    
    def read(self):
        time.sleep_ms(30)
        return self.sensor.read()
        
    
class TOFSensorArray:
    def __init__(self, config):
        
        #config = [
        #    (0, 8, 9),
        #    #() enter second sensor pins
        #]
        
        self.sensors = [
            TOFSensor(i2c_id, sda, scl) for (i2c_id, sda, scl) in config
        ]
        
        
    def read_left(self):
        return self.sensors[0].read() #assuming 0,8,9 corresponds to the left sensor
    
    def read_right(self):
        return self.sensors[1].read()
    


#in main.py: import TOFSensorArray, then tof_array = TOFSensorArray([],[])
#left_distance = tof_array.read_left()
#right_distance = tof_array.read_right()
