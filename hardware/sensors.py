from machine import Pin
import time

class IRSensorArray: 
    def __init__(self, pins, weights):
        self.sensors = [Pin(p, Pin.IN) for p in pins]
        self.weights = weights

    def read(self):
        time.sleep_ms(10)
        return [s.value() for s in self.sensors]
    
    def compute_error(self, vals):
        if vals in ([1,1,1,0], [0,1,1,1], [1,1,1,1]):
            return 0
        
        total, weight_sum = 0,0
        for w,v in zip(self.weights, vals):
            total += w * v
            weight_sum += v
        return 0 if weight_sum == 0 else total/weight_sum
    
    def detect_junction(self, vals):
        if vals == [1,1,1,0]: return 'LEFT'
        if vals == [0,1,1,1]: return 'RIGHT'
        if vals == [1,1,1,1]: return 'CROSS'
        return None
