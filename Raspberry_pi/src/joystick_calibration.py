from evdev import InputDevice, categorize, ecodes
from select import select
import numpy as np

class Joystick:
    def __init__(self , event):
        self.gamepad = InputDevice(event)    
        self.servo=0        
        self.pwm_referance = 4500
        self.save = False
        self.reset = False
    def read(self):
        r,w,x = select([self.gamepad.fd], [], [], 0.)
        
        if r:
            for event in self.gamepad.read():
#                print(event)
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        self.reset = False
                        self.save  = False
                        if event.code == 544:#up arrow
                            self.pwm_referance += 1
                        if event.code == 545:#down arrow
                            self.pwm_referance -= 1
                        if event.code == 547:#right arrow
                            self.pwm_referance += 10
                        if event.code == 546:#left arrow
                            self.pwm_referance -= 10   
                        if event.code == 307:#triangle
                            self.reset = True
                            self.pwm_referance=4500
                            self.save = False

                        if event.code == 308:#square
                            self.save = True
                        if event.code == 310:#R1
                            if(self.servo==11):
                                break
                            self.servo += 1
                            self.reset = True
                            self.pwm_referance=4500
                            self.save = False
              
                        if event.code == 311:#L1
                            if(self.servo==0):
                                break
                            self.servo-=1
                            self.reset = True
                            self.pwm_referance=4500
                            self.save = False
                    else:
                        print("boton down")
        return self.save,self.servo,self.pwm_referance,self.reset