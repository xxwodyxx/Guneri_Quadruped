import numpy as np
from numpy.lib.function_base import angle
import geometrics as geo



def Forward_R_kinematic(angles , coxa , femur , tabia): 
    """
    INPUT ANGLES --> RADIAN...
    """
    print(angles[0],angles[1],angles[2])
    print(coxa,femur,tabia)

    #BASE-LİNK1 ROTATİON AXİS MATRİX
    Transform_01  =  np.matrix([[   0,  0, -1, 0],
                                [   0,  1,  0, 0],
                                [   1,  0,  0, 0],
                                [   0,  0,  0, 1]])
    #LİNK1 TRANSFORM
    Transform_12 =   geo.RTmatrix(   np.array([0,    0,  angles[0]])   ,   np.array([coxa*np.sin(angles[0]),-coxa*np.cos(angles[0]),0]))
    
    #LİNK1-LİNK2 ROTATİON AXİS MATRİX
    Transform_23 =   np.matrix([[   1,  0,  0, 0],
                                [   0,  0, -1, 0],
                                [   0,  1,  0, 0],
                                [   0,  0,  0, 1]])

    #LİNK2 TRANSFORM
    Transform_34 =   geo.RTmatrix(   np.array([0,    0,  angles[1]])   ,   np.array([-femur*np.cos(angles[1]),-femur*np.sin(angles[1]),0]))

    #FOOT TRANSFORM
    Transform_45 =   geo.RTmatrix(   np.array([0,    0,  angles[2]])   ,   np.array([-tabia*np.cos(angles[2]),-tabia*np.sin(angles[2]),0]))

    Link_1_rot_xyz = Transform_01*Transform_12
    Link_2_rot_xyz = Transform_01*Transform_12*Transform_23*Transform_34
    Link_3_rot_xyz = Transform_01*Transform_12*Transform_23*Transform_34*Transform_45
    return np.round_(Link_1_rot_xyz),np.round_(Link_2_rot_xyz),np.round_(Link_3_rot_xyz)




def Forward_L_Kinematic(angles , coxa , femur , tabia): 
    """
    INPUT ANGLES --> RADIAN...
    """
    #BASE-LİNK1 ROTATİON AXİS MATRİX
    Transform_01  =  np.matrix([[   0,  0,  -1, 0],
                                [   0,  1,  0, 0],
                                [   1,  0,  0, 0],
                                [   0,  0,  0, 1]])
    #LİNK1 TRANSFORM
    Transform_12 =  geo.RTmatrix(   np.array([0,0,angles[0]])   ,   np.array([+coxa*np.sin(angles[0]),coxa*np.cos(angles[0]),0]))

    #LİNK1-LİNK2 ROTATİON AXİS MATRİX
    Transform_23 =   np.matrix([[   1,  0,  0, 0],
                                [   0,  0,  -1, 0],
                                [   0,  1,  0, 0],
                                [   0,  0,  0, 1]])

    #LİNK2 TRANSFORM
    Transform_34 =  geo.RTmatrix(   np.array([0,0,angles[1]])   ,   np.array([femur*np.cos(angles[1]),-femur*np.sin(angles[1]),0]))

    #FOOT TRANSFORM
    Transform_45 =  geo.RTmatrix(   np.array([0,0,angles[2]])   ,   np.array([-tabia*np.cos(angles[2]),-tabia*np.sin(angles[2]),0]))

    Link_1_rot_xyz = Transform_01*Transform_12
    Link_2_rot_xyz = Link_1_rot_xyz*Transform_23*Transform_34
    Link_3_rot_xyz = Link_2_rot_xyz*Transform_45

    return np.round_(Link_1_rot_xyz),np.round_(Link_2_rot_xyz),np.round_(Link_3_rot_xyz)