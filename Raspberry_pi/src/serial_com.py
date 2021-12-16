#!/usr/bin/env python3
import serial
import time
import numpy
from serial.serialutil import EIGHTBITS

class microcontroller_serial:  
    def __init__(self , port,boudrate):
        #ls -l /dev | grep ACM to identify serial port of the Stm
        self.Stm = serial.Serial(port, boudrate)
        # Nota: provocamos un reseteo manual de la placa para leer desde
        # el principio
        self.Stm.setDTR(False)
        time.sleep(1)
        self.Stm.flushInput()
        self.Stm.setDTR(True)
        time.sleep(2)
        
    def serialSend(self, pulse):  
        comando = "<r#{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}>"
        command=comando.format(int(pulse[0]), int(pulse[1]), int(pulse[2]), 
                                   int(pulse[3]), int(pulse[4]), int(pulse[5]), 
                                   int(pulse[6]), int(pulse[7]), int(pulse[8]), 
                                   int(pulse[9]), int(pulse[10]), int(pulse[11]))
        self.send_command=self.Stm.write(command.encode("UTF-8"))
        print(self.send_command)

        """
        while not(self.Stm.writable()):
            pass
        self.Stm.write(bytes(command , encoding='UTF-8'))
        print(command)
        """

    def serialRecive(self):
        try:
            startMarker = 60
            endMarker = 62
            
            getSerialValue = bytes()
            x = "z" # any value that is not an end- or startMarker
            byteCount = -1 # to allow for the fact that the last increment will be one too many
            
            # wait for the start character
            while  ord(x) != startMarker: 
                x = self.Stm.read()
                
                # save data until the end marker is found
            
            self.Stm.readable()
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x 
                    byteCount += 1
                x = self.Stm.read()
                
            loopTime , Xacc , Yacc , roll , pitch  = numpy.fromstring(getSerialValue.decode('ascii', errors='replace'), sep = '#' )
#             print(loopTime , roll , pitch) 

        except ValueError:
            loopTime = 0.
            roll = 0.
            pitch = 0.
            Xacc = 0.
            Yacc = 0.
            pass
            
        self.Stm.flushInput()    
        
        return loopTime , Xacc , Yacc , roll , pitch
            
    def close(self):
        self.Stm.close()