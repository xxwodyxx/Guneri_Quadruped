
import numpy as np

dosya = open("C:/Users/callo/Desktop/QUADRUPED/software/raspberry/data/pulse_referance.txt")
asd=dosya.read().split('#')
print(asd)
dosya.close()
print("---------------------------------------------")
dosya=open("C:/Users/callo/Desktop/QUADRUPED/software/raspberry/data/pulse_referance.txt","w")
new_pulse = "4250#4500#4500#4500#4500#4500#4500#4500#4500#4500#4500#4550"
dosya.write(new_pulse)
dosya.close()
dosya = open("C:/Users/callo/Desktop/QUADRUPED/software/raspberry/data/pulse_referance.txt")
print(dosya.readlines())

dosya.close()

