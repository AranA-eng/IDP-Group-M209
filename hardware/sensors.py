from machine import Pin, ADC, PWM, SoftI2C, I2C
from libs.VL53L0X.VL53L0X import VL53L0X
import time
from utime import sleep
from libs.tcs3472_micropython.tcs3472 import tcs3472
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701


class IRSensorArray: 
    def __init__(self, pins, weights):
        self.sensors = [Pin(p, Pin.IN) for p in pins]
        self.weights = weights

    def read(self):
        time.sleep(0.03)
        return [s.value() for s in self.sensors]
    
    def compute_error(self, vals):
        if vals in ([1,1,1,0], [0,1,1,1], [1,1,1,1]):
            return 0
        
        total, weight_sum = 0,0
        for w,v in zip(self.weights, vals):
            total += w * v
            weight_sum += v
        return 0 if weight_sum == 0 else total/weight_sum


class VL53L0X: # the VL53L0X dist sensor, expected config (0, 8, 9)
    def __init__(self, i2c):
        self.i2c = i2c
        self.sensor = VL53L0X(i2c)
'''   
    def read(self):
        self.sensor.start()
        sleep(0.1)
        distance = self.sensor.read()
        self.sensor.stop()
        return distance
'''

#in main.py: import TOFSensorArray, then tof_array = TOFSensorArray([],[])
#left_distance = tof_array.read_left()
#right_distance = tof_array.read_right()

class TCS34725: # the colour sensor
    def __init__(self, i2c, onoffpin: Pin, integration_time=0xEB, gain=0x01):
        self.i2c = i2c
        self.integration_time = integration_time
        self.gain = gain
        self.onoff = onoffpin
        
        # TCS34725 I2C address
        self.TCS34725_ADDR = 0x29
        self.COMMAND_BIT = 0x80

        # Register addresses
        self.REG_ENABLE = 0x00
        self.REG_ATIME = 0x01
        self.REG_CONTROL = 0x0F
        self.REG_ID = 0x12
        self.REG_CDATAL = 0x14  # Clear channel data low byte
        self.REG_RDATAL = 0x16
        self.REG_GDATAL = 0x18
        self.REG_BDATAL = 0x1A
        
        # Enable register bits
        self.ENABLE_AEN = 0x02  # RGBC enable
        self.ENABLE_PON = 0x01  # Power ON

        # Check sensor ID
        sensor_id = self._read8(self.REG_ID)
        if sensor_id not in (0x44, 0x10):
            raise RuntimeError("TCS34725 not found or wrong ID: 0x{:02X}".format(sensor_id))

        # Set integration time and gain
        self._write8(self.REG_ATIME, self.integration_time)
        self._write8(self.REG_CONTROL, self.gain)

        # Enable the device
        self.enable()

    def enable(self):
        self._write8(self.REG_ENABLE, self.ENABLE_PON)
        time.sleep_ms(3)
        self._write8(self.REG_ENABLE, self.ENABLE_PON | self.ENABLE_AEN)

    def disable(self):
        reg = self._read8(self.REG_ENABLE)
        self._write8(self.REG_ENABLE, reg & ~(self.ENABLE_PON | self.ENABLE_AEN))

    def _read8(self, reg):
        return self.i2c.readfrom_mem(self.TCS34725_ADDR, self.COMMAND_BIT | reg, 1)[0]

    def _read16(self, reg):
        data = self.i2c.readfrom_mem(self.TCS34725_ADDR, self.COMMAND_BIT | reg, 2)
        return data[1] << 8 | data[0]

    def _write8(self, reg, value):
        self.i2c.writeto_mem(self.TCS34725_ADDR, self.COMMAND_BIT | reg, bytes([value]))

    def read_raw(self):
        """Returns raw (clear, red, green, blue) values."""
        clear = self._read16(self.REG_CDATAL)
        red = self._read16(self.REG_RDATAL)
        green = self._read16(self.REG_GDATAL)
        blue = self._read16(self.REG_BDATAL)
        return clear, red, green, blue

    def calculate_color_temperature(self, r, g, b):
        """Approximate color temperature in Kelvin."""
        if r == 0 or g == 0 or b == 0:
            return 0
        X = (-0.14282 * r) + (1.54924 * g) + (-0.95641 * b)
        Y = (-0.32466 * r) + (1.57837 * g) + (-0.73191 * b)
        Z = (-0.68202 * r) + (0.77073 * g) + (0.56332 * b)
        if X + Y + Z == 0:
            return 0
        xc = X / (X + Y + Z)
        yc = Y / (X + Y + Z)
        n = (xc - 0.3320) / (0.1858 - yc)
        return int((449 * (n ** 3)) + (3525 * (n ** 2)) + (6823.3 * n) + 5520.33)

    def calculate_lux(self, r, g, b):
        """Approximate lux value."""
        return int((-0.32466 * r) + (1.57837 * g) + (-0.73191 * b))
    
    def read(self):
        self.onoff.value(1)
        self.enable()
        sleep(1)
        clear, red, green, blue = self.read_raw()
        self.disable()
        self.onoff.value(0)
        return clear, red, green, blue

"""
class TMF8701:
    def __init__(self, device: DFRobot_TMF8701):
        self.dev = device

    def read(self):
        
        while self.dev.begin() != 0:
            print("initialising")
            time.sleep(0.3)
        print("initialised")
        
        Returns distance in mm or None if not ready.
        self.dev.start_measurement(calib_m=self.dev.eMODE_NO_CALIB, mode=self.dev.eDISTANCE)
        
        while self.dev.is_data_ready() != True:
            sleep(0.02)

        #print("data ready")
        
        distance = 0
        
        while self.dev.get_distance_mm() == 0:
            distance = self.dev.get_distance_mm()
            
        self.dev.stop_measurement()

        return distance
"""
    

class TMF8701:
    def __init__(self, device: DFRobot_TMF8701):
        self.dev = device
        self.initialised = False

    def init(self):
        """Initialise & start measurement ONCE."""
        if self.initialised:
            return

        # Initialise
        while self.dev.begin() != 0:
            print("initialising")
            time.sleep(0.3)

        #print("TMF8701 initialised")

        # Start continuous measurement
        self.dev.start_measurement(
            calib_m=self.dev.eMODE_NO_CALIB,
            mode=self.dev.eDISTANCE
        )

        self.initialised = True

    def read(self):
        """Return a stable distance measurement in mm."""
        self.init()   # ensure initialised once

        # Only read when data is ready
        while not self.dev.is_data_ready():
            sleep(0.05)
        
        distance = self.dev.get_distance_mm()

        if distance == 0:
            while not self.dev.is_data_ready():
                d2 = self.dev.get_distance_mm()
                if d2 > 0:
                    return d2
                else:
                    #print("bad read")
                    self.read()
                    return 10000                                            #if we get a bad read, make the distance huge so that it doesn't pass the box detection test

        return distance
