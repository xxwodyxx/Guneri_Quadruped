import numpy as np
import time

from src.joystick_calibration import Joystick
from src.serial_com import microcontroller_serial

joystick = Joystick('/dev/input/event1') 

loopTime = 0.
interval = 0.040
servo_referance=[]
while True:
    if (time.time()-loopTime >= interval):
        loopTime = time.time() 
        
        print(joystick.read())

