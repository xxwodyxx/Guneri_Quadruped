#!/usr/bin/python3
import sys
import time
from PyQt5 import QtGui,QtCore,QtWidgets
from numpy.lib.function_base import angle
from robot_main import robot_main
import quadruped_form

class MainUiClass(QtWidgets.QMainWindow,quadruped_form.Ui_MainWindow):
   def __init__(self,parent= None):
        super(MainUiClass,self).__init__(parent)
        self.setupUi(self)
        self.threadclass = ThreadClass()
        self.threadclass.th_roll.connect(self.label_roll.setText)
        self.threadclass.th_pitch.connect(self.label_pitch.setText)
        self.threadclass.th_yaw.connect(self.label_yaw.setText)
        self.threadclass.th_speed.connect(self.label_v.setText)
        self.threadclass.th_angle.connect(self.label_angle.setText)
        self.threadclass.th_rotation.connect(self.label_rotation.setText)
        self.threadclass.th_time.connect(self.label_steptime.setText)
        self.threadclass.th_mode.connect(self.label_mode.setText)
        self.threadclass.th_style.connect(self.label_gaitstyle.setText)
        self.threadclass.th_compliant.connect(self.label_compliantmode.setText)
        self.threadclass.th_xacc.connect(self.label_xacc.setText)
        self.threadclass.th_yacc.connect(self.label_yacc.setText)
        self.threadclass.th_zacc.connect(self.label_zacc.setText)
        self.threadclass.th_errorc.connect(self.label_errorcounter.setText)
        self.threadclass.th_ikdomain.connect(self.label_domainerror.setText)
        self.threadclass.th_startfunc.connect(self.label_startfunction.setText)
        self.threadclass.th_resetfunc.connect(self.label_resetfunction.setText)
        self.threadclass.th_battery.connect(self.battery_voltage_bar.setValue)
        self.threadclass.th_ps3_connect.connect(self.label_ps3connect.setText)
        self.threadclass.th_serial_connect.connect(self.label_serialconnect.setText)
        self.button_start.clicked.connect(self.button_start_func)
        self.button_stop.clicked.connect(self.button_stop_func)
        self.show()


   def button_stop_func(self):
      self.threadclass.terminate()


   def button_start_func(self):
      self.threadclass.start()


class ThreadClass(QtCore.QThread,robot_main):
   th_roll     =   QtCore.pyqtSignal(str)
   th_pitch    =   QtCore.pyqtSignal(str)
   th_yaw      =   QtCore.pyqtSignal(str)
   th_speed    =   QtCore.pyqtSignal(str)
   th_angle    =   QtCore.pyqtSignal(str)
   th_rotation =   QtCore.pyqtSignal(str)
   th_time     =   QtCore.pyqtSignal(str)
   th_mode     =   QtCore.pyqtSignal(str)
   th_style    =   QtCore.pyqtSignal(str)
   th_compliant=   QtCore.pyqtSignal(str)
   th_xacc     =   QtCore.pyqtSignal(str)
   th_yacc     =   QtCore.pyqtSignal(str)
   th_zacc     =   QtCore.pyqtSignal(str)
   th_errorc   =   QtCore.pyqtSignal(str)
   th_ikdomain =   QtCore.pyqtSignal(str)
   th_startfunc=   QtCore.pyqtSignal(str)
   th_resetfunc =   QtCore.pyqtSignal(str)
   th_ps3_connect= QtCore.pyqtSignal(str)
   th_serial_connect = QtCore.pyqtSignal(str)
   th_battery  =   QtCore.pyqtSignal(int)

   def __init__(self, parent = None):
      super(ThreadClass,self).__init__(parent)
      self.setup_init()

   def run(self):
        while True:
            self.robot_run()
            self.emit_func([self.th_roll,    str(self.commandOrn[0])],
                           [self.th_pitch,   str(self.commandOrn[1])],
                           [self.th_yaw,     str(self.commandOrn[2])],
                           [self.th_speed,   str(self.V)],
                           [self.th_angle,   str(self.angle)],
                           [self.th_rotation,str(self.Wrot)],
                           [self.th_time,    str(self.T)],
                           [self.th_mode,    str(self.mode[self.mode_value])],
                           [self.th_style,   str(self.gait_style[self.gait_style_value][0])],
                           [self.th_compliant,str(self.compliantMode)],
                           [self.th_xacc,    str(self.commandPose[0])],
                           [self.th_yacc,    str(self.commandPose[1])],
                           [self.th_zacc,    str(self.commandPose[2])],
                           [self.th_errorc,  str(self.error)],
                           [self.th_ikdomain,str(self.ik__error)],
                           [self.th_startfunc,str(self.startfunc)],
                           [self.th_resetfunc,str(self.resetfunc)],
                           [self.th_battery, int(self.battery_vol)], 
                           [self.th_ps3_connect,self.jyconnect],
                           [self.th_serial_connect,self.stmconect])
            
   def emit_func(self,*args):
      for value in args:
         value[0].emit(value[1])

   
if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv) 
    MainWindow = MainUiClass()
    sys.exit(app.exec_())
    
