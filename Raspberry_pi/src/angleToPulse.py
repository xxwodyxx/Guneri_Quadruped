import numpy as np


class angleToPulse:
    def __init__(self,referance_adress):
        with open(referance_adress,"r") as serv_referance_file:
            self.reference_pulse = serv_referance_file.read().split('#')


    def convert(self,FR_angles, FL_angles, BR_angles, BL_angles):
        min_pulse = 1500        
        max_pulse = 7500
        pulse_deg_ratio = (max_pulse-min_pulse)/180
        pulse = np.empty([12])
        
        #FR
        pulse[0]    = int(  np.rad2deg(-FR_angles[0])        *pulse_deg_ratio + int(self.reference_pulse[0]))
        pulse[1]    = int(  np.rad2deg(FR_angles[1])        *pulse_deg_ratio + int(self.reference_pulse[1]))
        pulse[2]    = int(  np.rad2deg(-FR_angles[2])       *pulse_deg_ratio + int(self.reference_pulse[2]))
        #FL
        pulse[3]    = int(  np.rad2deg(+FL_angles[0])       *pulse_deg_ratio + int(self.reference_pulse[3])) 
        pulse[4]    = int(  np.rad2deg(-FL_angles[1])       *pulse_deg_ratio + int(self.reference_pulse[4]))
        pulse[5]    = int(  np.rad2deg(FL_angles[2])        *pulse_deg_ratio + int(self.reference_pulse[5]))
        #BR
        pulse[6]    = int(  np.rad2deg(-BR_angles[0])        *pulse_deg_ratio + int(self.reference_pulse[6]))
        pulse[7]    = int(  np.rad2deg(BR_angles[1])        *pulse_deg_ratio + int(self.reference_pulse[7])) 
        pulse[8]    = int(  np.rad2deg(BR_angles[2])        *pulse_deg_ratio + int(self.reference_pulse[8]))
        #BL
        pulse[9]    = int(  np.rad2deg(+BL_angles[0])       *pulse_deg_ratio + int(self.reference_pulse[9]))
        pulse[10]   = int(  np.rad2deg(-BL_angles[1])       *pulse_deg_ratio + int(self.reference_pulse[10]))
        pulse[11]   = int(  np.rad2deg(-BL_angles[2])       *pulse_deg_ratio + int(self.reference_pulse[11]))
        return pulse