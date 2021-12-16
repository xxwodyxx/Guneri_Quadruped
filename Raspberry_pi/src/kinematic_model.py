import numpy as np
import src.geometrics as geo
import src.IK_solver as IK


"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/_____________  x       "
"""

class robotKinematics:
    def __init__(self):
        """in meter """
        self.L = 0.20 #length of robot joints       20cm
        self.W = 0.08 #width of robot joints         8cm
        self.coxa = 0.05#coxa length                 5cm   
        self.femur = 0.10#femur length              10cm    
        self.tibia = 0.10#tibia length              10cm
        """initial foot position"""
        #foot separation (0.182 -> tetta=0) and distance to floor
        self.bodytoFR0 = np.array([ self.L/2, -self.W/2 , 0])
        self.bodytoFL0 = np.array([ self.L/2,  self.W/2 , 0])
        self.bodytoBR0 = np.array([-self.L/2, -self.W/2 , 0])
        self.bodytoBL0 = np.array([-self.L/2,  self.W/2 , 0])
        #body frame to foot frame vector
        self.ik_error=0
    def solve(self, orn , pos , bodytoFeet):
        bodytoFR4 = np.asarray([bodytoFeet[0,0],bodytoFeet[0,1],bodytoFeet[0,2]])
        bodytoFL4 = np.asarray([bodytoFeet[1,0],bodytoFeet[1,1],bodytoFeet[1,2]])
        bodytoBR4 = np.asarray([bodytoFeet[2,0],bodytoFeet[2,1],bodytoFeet[2,2]])
        bodytoBL4 = np.asarray([bodytoFeet[3,0],bodytoFeet[3,1],bodytoFeet[3,2]])

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = geo.transform(self.bodytoFR0 , orn, pos)
        _bodytoFL0 = geo.transform(self.bodytoFL0 , orn, pos)
        _bodytoBR0 = geo.transform(self.bodytoBR0 , orn, pos)
        _bodytoBL0 = geo.transform(self.bodytoBL0 , orn, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""
        undoOrn = -orn
        undoPos = -pos
        _FRcoord = geo.transform(FRcoord , undoOrn, undoPos)
        _FLcoord = geo.transform(FLcoord , undoOrn, undoPos)
        _BRcoord = geo.transform(BRcoord , undoOrn, undoPos)
        _BLcoord = geo.transform(BLcoord , undoOrn, undoPos)
        
        
        FR_angles,self.ik_error = IK.solve_R(_FRcoord , self.coxa , self.femur , self.tibia,self.ik_error)
        FL_angles,self.ik_error = IK.solve_L(_FLcoord , self.coxa , self.femur , self.tibia,self.ik_error)
        BR_angles,self.ik_error = IK.solve_R(_BRcoord , self.coxa , self.femur , self.tibia,self.ik_error)
        BL_angles,self.ik_error = IK.solve_L(_BLcoord , self.coxa , self.femur , self.tibia,self.ik_error)
        
        _bodytofeetFR = _bodytoFR0 + _FRcoord
        #print(f"FR BODY--> {_bodytoFR0}")
        #print(f"FR--> {_FRcoord}")
        _bodytofeetFL = _bodytoFL0 + _FLcoord
        #print(f"FL BODY--> {_bodytoFL0}")
        #print(f"FL--> {_FLcoord}")
        _bodytofeetBR = _bodytoBR0 + _BRcoord
        #print(f"BR BODY--> {_bodytoBR0}")
        #print(f"BR--> {_BRcoord}")
        _bodytofeetBL = _bodytoBL0 + _BLcoord
        #print(f"BL BODY--> {_bodytoBL0}")
        #print(f"BL--> {_BLcoord}")
        _bodytofeet = np.matrix([[_bodytofeetFR[0] , _bodytofeetFR[1] , _bodytofeetFR[2]],
                                 [_bodytofeetFL[0] , _bodytofeetFL[1] , _bodytofeetFL[2]],
                                 [_bodytofeetBR[0] , _bodytofeetBR[1] , _bodytofeetBR[2]],
                                 [_bodytofeetBL[0] , _bodytofeetBL[1] , _bodytofeetBL[2]]])
        
        #print(f"FR-->{np.round_(FR_angles*180.0/np.pi,2)}|FL-->{np.round_(FL_angles*180.0/np.pi,2)}|BR-->{np.round_(BR_angles*180.0/np.pi,2)}|BL-->{np.round_(BL_angles*180.0/np.pi,2)}")
        return FR_angles, FL_angles, BR_angles,BL_angles,self.ik_error

        #return f"FR-->{np.round_(FR_angles*180.0/np.pi,5)}, FL-->{np.round_(FL_angles*180.0/np.pi,5)}, BR-->{np.round_(BR_angles*180.0/np.pi,5)}, BL-->{np.round_(BL_angles*180.0/np.pi,5)}"
