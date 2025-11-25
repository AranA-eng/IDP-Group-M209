#set the standard speed to 50 then calculate heights off that
#reset lowers the actuator to a minimum then raises slightly so the fork is just off the ground (true 0 datum)

from machine import Pin, PWM
from utime import sleep

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        self.height = 0
           
    def set(self, dir, duration):
        speed = 50 # standard speed 50
        self.mDir.value(dir)                     # forward = 1, reverse = 0 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor
        sleep(duration)
        self.stop()
        sleep(1)
        
    def reset(self):
        self.set(0, 10)
        self.set(1, 0.8) #set back to 0cm datum
        self.height = 0
        
    def setheight(self, newheight): # eg if height was from 0 to 10
        if newheight > 35: #saturate the heights in mm
            newheight = 35
        elif newheight < 0:
            newheight = 0
            
        distance = newheight - self.height
        timeon = abs(distance/3) #translate from distance (mm) to time actuator is on (1 time unit = 3mm)
        
        if distance == 0:
            return
        if distance > 0: #forward = upwards = +ve height diff
            self.set(1, timeon)
        else:
            self.set(0, timeon)
            
        self.height = newheight
            
    def stop(self):
        self.pwm.duty_u16(0)



#test -----------------------------------------
def test_actuator1():
    actuator1 = Actuator(dirPin=0, PWMPin=1)  # Actuator 1 controlled from Motor Driv1 #1, which is on GP0/1
    actuator1.reset()


    actuator1.setheight(27)
    sleep(5)
    actuator1.setheight(37)
    
    try:
        while True:
            actuator1.setheight(27)
            sleep(10)
            actuator1.setheight(34)
            
    except KeyboardInterrupt:
        #actuator1.stop()
    
    
if __name__ == "__main__":
    print("testing")
    test_actuator1()
    

