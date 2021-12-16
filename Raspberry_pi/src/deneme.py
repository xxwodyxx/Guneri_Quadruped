import numpy as np
import kinematic_model as km


kinematik=km.robotKinematics()

orn = np.array([ 0,      np.pi/36,     0])
pos = np.array([ 0.03,      0,     0])  

Xdist =0.19
Ydist =0.07
height=0.12
coxa = 0.05

bodytoFeet = np.array([[ Xdist/2 , -Ydist/2-coxa , -height],     #FR
                       [  Xdist/2 ,  Ydist/2+coxa , -height],     #FL
                       [ -Xdist/2 , -Ydist/2-coxa , -height],     #BR
                       [ -Xdist/2 ,  Ydist/2+coxa , -height]])    #BL

print(kinematik.solve(orn , pos , bodytoFeet))
