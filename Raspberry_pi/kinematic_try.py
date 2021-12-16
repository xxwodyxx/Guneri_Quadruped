import numpy as np
import src.kinematic_model as km
import src.angleToPulse as pul

kinematik=km.robotKinematics()

orn = np.array([ 0,         0,     np.pi/18])
pos = np.array([ 0,         0,     0])  

Xdist =0.20
Ydist =0.10
height=0.15

bodytoFeet = np.array([[ Xdist/2 ,  -Ydist/2-kinematik.coxa , -height],     #FR
                       [  Xdist/2 ,  Ydist/2+kinematik.coxa , -height],     #FL
                       [ -Xdist/2 , -Ydist/2-kinematik.coxa , -height],     #BR
                       [ -Xdist/2 ,  Ydist/2+kinematik.coxa , -height]])    #BL
angles = kinematik.solve(orn , pos , bodytoFeet)

print(pul.convert(angles[0],angles[1],angles[2],angles[3]))

#print(kinematik.solve(orn , pos , bodytoFeet))
