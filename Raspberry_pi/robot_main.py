import numpy as np
import time
from src.kinematic_model import robotKinematics
from src.joystick import Joystick
from src.serial_com1 import microcontroller_serial
from src.angleToPulse import angleToPulse
from src.gaitPlanner import trotGait

class robot_main:
    def __init__(self) -> None:
        self.Ydist = 0.08
        self.Xdist = 0.20
        self.height = 0.15
        self.bodytoFeet0 = np.array([[ self.Xdist/2 , -self.Ydist/2-0.05 , -self.height],     #FR
                                    [  self.Xdist/2 ,  self.Ydist/2+0.05 , -self.height],     #FL
                                    [ -self.Xdist/2 , -self.Ydist/2-0.05 , -self.height],     #BR
                                    [ -self.Xdist/2 ,  self.Ydist/2+0.05 , -self.height]])    #BL
        self.orn = np.array([0. , 0. , 0.])
        self.pos = np.array([0. , 0. , 0.])
        self.Upid_yorn = [0.]
        self.Upid_y = [0.]
        self.Upid_xorn = [0.]
        self.Upid_x = [0.]
        self.startTime = time.time()
        self.lastTime = self.startTime
        self.T = 0.4 #period of time (in seconds) of every step
        self.gait_style=   [["Creep" ,  np.array([0. , 0.25 , 0.75 , 0.5])],
                            ["Walk"  ,  np.array([0. , 0.25 , 0.75 , 0.5])],
                            ["Trot"  ,  np.array([0. , 0.25 , 0.75 , 0.5])],
                            ["Gallop",  np.array([0. , 0.25 , 0.75 , 0.5])]]
        self.interval = 0.02
        self.exit_all = False
        self.error = 0
        self.acceleration_array = np.array([0,0,0])
        self.ik__error = 0
        self.startfunc = False
        self.resetfunc = False
        self.compliantMode = False
        self.V = 0
        self.angle = 0
        self.Wrot = 0
        self.T = 0.4
        self.battery_vol = 0
        self.gait_style_value=0
        self.mode_value = 0
        self.mode =["Gait","Position","Orientation"]
        self.commandPose = np.array([0. , 0. , 0.])
        self.commandOrn = np.array([0. , 0. , 0.])
        self.jyconnect = "NOT Connected"
        self.stmconect = "NOT Connected"
        
    def setup_init(self):
        self.robotKinematics = robotKinematics()
        self.convert_pulse=angleToPulse("/home/pi/Quadruped/software/raspberry/data/pulse_referance.txt")
        self.trot = trotGait() 
        while True:
            try:
                self.joystick = Joystick('/dev/input/event1') #need to specify the event route
                self.jyconnect = "Connected" 
                print("PS3 joystick Connected.")
                break
            except:
                self.jyconnect = "NOT Connected"
                print("PS3 joystick NOT Connected...")
                time.sleep(1)
        while True:
            try :
                self.stm = microcontroller_serial('/dev/ttyS0',115200) #need to specify the serial port
                self.stmconect = "Connected"
                print("Serial Connected.")
                break
            
            except:
                self.stmconect = "NOT Connected"
                print("Serial Communication NOT Connected")
                time.sleep(1)
            
    def robot_run(self):
        #try:
        if True:
            if (time.time()-self.lastTime >= self.interval):
                self.loopTime = time.time() - self.lastTime
                self.lastTime = time.time()
                self.commandPose,self.commandOrn,self.V,self.angle,self.Wrot,self.T,self.compliantMode,self.mode_value,self.gait_style_value,self.resetfunc,self.startfunc,self.exit_all=self.joystick.read()
                self.bodytoFeet  = self.trot.loop(self.V  , self.angle  , self.Wrot , self.T , self.gait_style[self.gait_style_value][1] , self.bodytoFeet0)
                self.FR_angles, self.FL_angles, self.BR_angles, self.BL_angles,self.ik__error = self.robotKinematics.solve(self.orn + self.commandOrn, self.pos + self.commandPose , self.bodytoFeet)
                self.pulsesCommand = self.convert_pulse.convert(self.FR_angles, self.FL_angles, self.BR_angles, self.BL_angles)
                
                if self.resetfunc:
                    self.stm.serial_close()
                    self.stm.serial_open()
                    return 0
                if self.startfunc:
                    self.stm.serialSend(self.pulsesCommand)

                if self.exit_all:
                    self.stm.serial_close()
                    self.joystick.gamepad.close()
                    exit()
        """
        except KeyboardInterrupt:
            exit()
        except:
            self.error = self.error+1
        """ 

            
        