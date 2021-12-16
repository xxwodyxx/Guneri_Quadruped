#!/usr/bin/env python3
import pigpio
import time
import numpy

class microcontroller_serial:  
    def __init__(self , port,baudrate):
        #ls -l /dev | grep ACM to identify serial port of the Stm
        self.pi = pigpio.pi()     # connected error aperance --> write terminal "sudo pigpiod"
        if not self.pi.connected:
            exit()
        self.baudrate = baudrate
        self.port = port
        self.Stm=self.pi.serial_open(self.port,self.baudrate)
        time.sleep(1)
                
    def serialSend(self, pulse):  
        comando = "<w#{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}>"
        command=comando.format(int(pulse[0]), int(pulse[1]), int(pulse[2]), 
                                   int(pulse[3]), int(pulse[4]), int(pulse[5]), 
                                   int(pulse[6]), int(pulse[7]), int(pulse[8]), 
                                   int(pulse[9]), int(pulse[10]), int(pulse[11]))
        lenght_string = len(command)
        mystring = command.encode('UTF-8')+ '\0'.encode("UTF-8")*(100-lenght_string)

        self.pi.serial_write(self.Stm,mystring)
    def serialRecive(self):
        try:
            startMarker = 60
            endMarker = 62
            
            getSerialValue = bytes()
            x = "z" # any value that is not an end- or startMarker
            byteCount = -1 # to allow for the fact that the last increment will be one too many
            
            # wait for the start character
            while  (ord(x) != startMarker): 
                b,x = self.pi.serial_read(self.Stm,1)
                
            
            #self.pi.serial_data_available() #return rx buffer lenght 
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x 
                    byteCount += 1 
                x = self.pi.serial_read(self.Stm,1)
                
            loopTime , Xacc , Yacc , roll , pitch  = numpy.fromstring(getSerialValue.decode('ascii', errors='replace'), sep = '#' )
#             print(loopTime , roll , pitch) 

        except ValueError:
            loopTime = 0.
            roll = 0.
            pitch = 0.
            Xacc = 0.
            Yacc = 0.
            pass
                    
        return loopTime , Xacc , Yacc , roll , pitch
            
    def serial_close(self):
        self.pi.serial_close(self.Stm)

    def serial_open(self):
        self.Stm=self.pi.serial_open(self.port,self.baudrate)
        time.sleep(1)


if(__name__ == "__main__"):
    uart = microcontroller_serial("/dev/ttyS0",115200)

    mylist = [4500,4500,4500,4500,4500,4500,4500,4500,4500,4500,4500,4500]
    uart.serialSend(mylist)
    uart.serial_close()

    