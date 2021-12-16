#!/usr/bin/env python3

from evdev import InputDevice, categorize, ecodes
from select import select
import numpy as np
from numpy.core.arrayprint import format_float_scientific



class Joystick:
    def __init__(self , event):
        #python3 /usr/local/lib/python3.8/dist-packages/evdev/evtest.py for identify event
        self.gamepad = InputDevice(event)
        self.L3 = np.array([0. , 0.])
        self.R3 = np.array([0. , 0.])
        self.x=0
        self.triangle=0
        self.circle=0
        self.square=0
        self.select_button = False
        self.start_button = False
        self.exit = False
        self.T = 0.4
        self.V = 0.
        self.angle = 0.
        self.Wrot = 0.
        self.compliantMode = False
        self.gait_Mode = True  # gait_Mode  0->walk    1->trot     2->     3->gallop
        self.pose_ornMode = False
        self.pose_posMode = False
        self.CoM_pos = np.zeros(3)
        self.CoM_orn = np.zeros(3)
        self.calibration = 0
        print("Gait mode has been activated...")
        print("\n")

    def read(self):
        r,w,x = select([self.gamepad.fd], [], [], 0.)
        
        if r:
            for event in self.gamepad.read():
#                print(event)
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == 544:           #up arrow
                            self.CoM_pos[2] += 0.002
                        if event.code == 545:           #down arrow
                            self.CoM_pos[2] -= 0.002
                        if event.code == 547:           #right arrow
                            self.T += 0.05
                        if event.code == 546:           #left arrow
                            self.T -= 0.05   

                        if event.code == 308:           #square
                            if self.compliantMode == True:
                                self.compliantMode = False
                            elif self.compliantMode == False:
                                self.compliantMode = True  
                        if event.code == 307:           #triangle
                            self.pose_ornMode = True
                            self.pose_posMode = False
                            self.gait_Mode = False
                            self.mode_value=2
                        if event.code == 305:           #circle
                            self.pose_ornMode = False
                            self.pose_posMode = True
                            self.gait_Mode = False
                            self.mode_value=1
                        if event.code == 304:           #X
                            self.pose_ornMode = False
                            self.pose_posMode = False
                            self.gait_Mode = True
                            self.mode_value=0
                        if event.code ==314:            #select
                            if self.select_button == True:      #reset button serial
                                self.select_button = False
                            elif self.select_button == False:
                                self.select_button = True  
                            print(f"-------->Reset mode : {self.select_button}<-------")
                        if event.code ==315:            #start 
                            if self.start_button == True:
                                self.start_button = False
                            elif self.start_button == False:
                                self.start_button = True  
                            print(f"-------->Start mode : {self.start_button}<-------")
                        if event.code == 316:           #P3 button
                            self.exit =True                          
                            
                        if event.code == 310:           #L1
                            self.gait_style_value += 1
                            if self.gait_style_value > 3 :
                                self.gait_style_value = 3
                        if event.code == 312:           #L2
                            self.gait_style_value -= 1
                            if self.gait_style_value < 0 :
                                self.gait_style_value = 0
                        if event.code == 313:           #R2
                            pass
                            #self.calibration -= 0.0005
                        if event.code == 311:           #R1
                            pass
                            #self.calibration -= 5
                    else:
                        print("botton errors")
                ########################################  for my own joystick
                #      ^           #     ^            #
                #    ABS_Y         #    ABS_RY        #
                #  ←─────→ ABS_X #  ←─────→ ABS_RX   #
                #     ↓           #     ↓            #  
                #######################################
                elif event.type == ecodes.EV_ABS:
                    absevent = categorize(event)
                    if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":  
                        self.L3[0]=absevent.event.value-127
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                        self.L3[1]=absevent.event.value-127
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                        self.R3[0]=absevent.event.value-127
#                        print(self.d_z)
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                        self.R3[1]=absevent.event.value-127
        
        if self.gait_Mode == True:           
            self.V = np.sqrt(self.L3[1]**2 + self.L3[0]**2)/100.
            self.angle = np.rad2deg(np.arctan2(-self.L3[0] , -self.L3[1]))
            self.Wrot = -self.R3[0]/250.
    #        Lrot = 0.
            if self.V <= 0.025:
                self.V = 0.
            if self.Wrot <= 0.025 and self.Wrot >= -0.025:
                self.Wrot = 0.
            self.CoM_pos[0],self.CoM_pos[1] = 0,0
            self.CoM_orn = np.zeros(3)

        elif self.pose_ornMode==True :
            self.CoM_orn[0] = np.deg2rad(self.R3[0]/3)
            self.CoM_orn[1] = np.deg2rad(self.L3[1]/3)
            self.CoM_orn[2] = -np.deg2rad(self.L3[0]/3)
            self.CoM_pos[0] = -self.R3[1]/5000
            self.V = 0
            self.angle = 0
            self.Wrot = 0

        elif self.pose_posMode ==True:
            self.CoM_pos[0] =  self.L3[1]/5000
            self.CoM_pos[1] = -self.L3[0]/5000
            self.CoM_pos[2] = -self.R3[1]/5000
            self.V = 0
            self.angle = 0
            self.Wrot = 0
        return self.CoM_pos , self.CoM_orn , self.V , -self.angle , -self.Wrot , self.T , self.compliantMode,self.mode_value,self.gait_style_value,self.select_button,self.start_button,self.exit
