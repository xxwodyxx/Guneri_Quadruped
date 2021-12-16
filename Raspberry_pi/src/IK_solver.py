from cv2 import error
import numpy

def checkdomain(D,errors):
    if D > 1 or D < -1:
        errors = errors + 1
        if D > 1: 
            D = 0.99
            return D , errors
        elif D < -1:
            D = -0.99
            return D , errors
    else:
        return D,errors
#this is based on this paper: 
#"https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/____________  x       "
"""
#IK equations now written in pybullet frame.
def solve_R(coord , coxa , femur , tibia,ik_error): 
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D,ik_error= checkdomain(D,ik_error)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([+(numpy.pi/2+gamma), +alpha, -tetta])
    return angles,ik_error

def solve_L(coord , coxa , femur , tibia,ik_error):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D,ik_error= checkdomain(D,ik_error)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([+(numpy.pi/2+gamma), +alpha, +tetta])
    return angles,ik_error

