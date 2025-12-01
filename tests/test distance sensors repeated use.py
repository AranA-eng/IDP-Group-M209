from machine import Pin, I2C
from libs.VL53L0X.VL53L0X import VL53L0X
from utime import sleep
#from hardware import sensors as sen
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701


def i2c_clear(scl_pin, sda_pin):
    scl = Pin(scl_pin, Pin.OUT, value=1)
    sda = Pin(sda_pin, Pin.OUT, value=1)
    for _ in range(9):
        scl.low(); sleep(0.001)
        scl.high(); sleep(0.001)
    # STOP
    sda.low(); sleep(0.001)
    scl.high(); sleep(0.001)
    sda.high(); sleep(0.001)


def test_vl53l0x():
    i2c_bus = I2C(id=0, sda=Pin(8), scl=Pin(9))
    
    # Setup vl53l0 object
    vl53l0 = VL53L0X(i2c_bus)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)


    vl53l0.start()
    
    for i in range(3):
        distance = vl53l0.read()
        print(f"Distance = {distance}mm")  # Check calibration!
        sleep(0.5)
        
    vl53l0.stop()

    return




#IR_PINS = [13, 12, 11, 10]
#line_sensor_weights = [-5.0, -2.0, 2.0, 5.0]
#IR_sensors = sen.IRSensorArray(IR_PINS, line_sensor_weights)

def test_TMF8x01_get_distance():
    i2c_bus = I2C(id=0, sda=Pin(20), scl=Pin(21), freq=100000) # I2C0 on GP8 & GP9


    # Set the correct device
    device = "TMF8701"


    # Use the correct one - TODO can we auto detect this?
    if device == "TMF8701":
      tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    elif device == "TMF8801":
      tof = DFRobot_TMF8801(i2c_bus=i2c_bus)
    else:
       raise RuntimeError(f"Device {device} not known")

    print("Initialising ranging sensor TMF8x01......")
    while(tof.begin() != 0):
      print("   Initialisation failed")
      sleep(0.2)
    print("   Initialisation done.")

    print("Software Version: ", end=" ")
    print(tof.get_software_version())
    print("Unique ID: %X"%tof.get_unique_id())
    print("Model: ", end=" ")
    print(tof.get_sensor_model())

    if device == "TMF8701":
      #tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.ePROXIMITY)
      #tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.eCOMBINE)
      tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.eDISTANCE)
    elif device == "TMF8801":
      tof.start_measurement(calib_m = tof.eMODE_NO_CALIB)
    else:
       raise RuntimeError(f"Device {device} not known")

    sleep(2)

    for i in range(3):
      if(tof.is_data_ready() == True):
        #print(IR_sensors.read())
        print(f"Distance = {tof.get_distance_mm()} mm{" (For TMF8701, make sure you read about mode selection above!)" if device == "TMF8701" else ""}")
      sleep(0.5)
      
    tof.stop_measurement()
    sleep(2)
    
    


if __name__ == "__main__":
    while True:
        print("TMF testing")
        test_TMF8x01_get_distance()
        i2c_clear(20, 21)
        print("VL testing")
        test_vl53l0x()
        i2c_clear(8, 9)