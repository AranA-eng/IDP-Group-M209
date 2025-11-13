import time

class PID:
    """
    Discrete PID controller
    usage: 
        pid = PID(kp = 1, ki = 1, kd= 1, out_min = -base_speed, out_max = base_speed)
        while True:
            control = pid.update(err)
    dt_ms is in ms
    """
    def __init__(self, kp, ki, kd, dt_ms, out_min = -1e9, out_max = 1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt_ms / 1000
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