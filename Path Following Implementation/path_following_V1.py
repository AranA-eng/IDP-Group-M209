from machine import Pin
import time

class PID:
    def __init__(self, kp, ki, kd, dt_pid, out_min = -1e9, out_max = 1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt_pid
        self.integral = 0.0
        self.prev_err = 0.0
        self.out_min, self.out_max = out_min, out_max
        
    
    def update(self, err):
        self.integral += err*self.dt
         
        deriv = (err - self.prev_err) / self.dt if self.dt > 0 else 0.0
        
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        
        if out > self.out_max:
            out = self.out_max
            
        elif out < self.out_min:
            out = self.out_min
        self.prev_err = err
        return out


