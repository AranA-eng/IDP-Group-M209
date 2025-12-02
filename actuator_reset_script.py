from hardware import sensors as sen
from hardware import linear_actuator as act

dirPin = 0
PWMPin = 1

actuator = act.Actuator(dirPin, PWMPin)
actuator.reset()
