#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import time

from src.serial_com import microcontroller_serial
from src.angleToPulse import angleToPulse

arduino = microcontroller_serial("/dev/ttyS0") #need to specify the serial port
angletopulse = angleToPulse("C:/Users/callo/Desktop/QUADRUPED/software/raspberry/data/pulse_referance.txt")

BR_angles = np.array([0 , 0 , -np.pi/2])#BR
BL_angles = np.array([0 , 0 , -np.pi/2])#BL
FL_angles = np.array([0 , 0 , -np.pi/2])#FL
FR_angles = np.array([0 , 0 , -np.pi/2])#FR

loopTime = 0.
interval = 0.040
while True:
    if (time.time()-loopTime >= interval):
        loopTime = time.time() 

        pulsesCommand = angletopulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
            
        arduino.serialSend(pulsesCommand)   #send serial command to arduino
