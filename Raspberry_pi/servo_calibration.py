import numpy as np
import time

from src.joystick_calibration import Joystick
from src.serial_com1 import microcontroller_serial

Stm = microcontroller_serial('/dev/ttyS0',115200) 
joystick = Joystick('/dev/input/event1') 
"""
Device 09:37:59:8D:63:25 Sony PLAYSTATION(R)3 Controller
Authorize service
"""

loopTime = 0.
interval = 0.05
servo_referance=[]
with open("/home/pi/Quadruped/software/raspberry/data/pulse_referance.txt","r") as serv_referance_file:
    pulsesCommand = serv_referance_file.read().split('#')
    servo_referance = pulsesCommand

while True:
    if (time.time()-loopTime >= interval):
        loopTime = time.time() 
        
        save,servo,pwm_referance,reset = joystick.read()
        pulsesCommand[servo]=int(pwm_referance)
        Stm.serialSend(pulsesCommand) 
        print(f" Servo -->{servo}  |  Pulse -->{pwm_referance}")
        if(save == True):
            with open("/home/pi/Quadruped/software/raspberry/data/pulse_referance.txt","w") as serv_referance_file:
                comando = "{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}" #Input
                command=comando.format( int(servo_referance[0]), int(servo_referance[1]), int(servo_referance[2]), 
                                        int(servo_referance[3]), int(servo_referance[4]), int(servo_referance[5]), 
                                        int(servo_referance[6]), int(servo_referance[7]), int(servo_referance[8]), 
                                        int(servo_referance[9]), int(servo_referance[10]), int(servo_referance[11]))
                serv_referance_file.write(command)
            with open("/home/pi/Quadruped/software/raspberry/data/pulse_referance.txt","r") as serv_referance_file:
                pulsesCommand = serv_referance_file.read().split('#')
                servo_referance = pulsesCommand
        if(reset==True):
            with open("/home/pi/Quadruped/software/raspberry/data/pulse_referance.txt","r") as serv_referance_file:
                pulsesCommand = serv_referance_file.read().split('#')
                servo_referance = pulsesCommand


"""
AngletoPulse --> Txt dosyasındaki referans servo pwm pulse'lerini ayarlanması kaldı.Sonra Robotta denenebilir.

"""