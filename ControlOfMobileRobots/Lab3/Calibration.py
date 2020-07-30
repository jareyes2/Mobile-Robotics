#calibration.py
#This progeam is stepping through each PWM from 1.3 - 1.7 at intervals of 0.005
#Saves results in a file
import time
import encoder_lib as enc

# Calibrate encoders  
enc.initEncoders()
timeX = time.monotonic()
enc.setSpeedsPWM(1.5, 1.5)
enc.calibrateSpeeds(1,'CalibrateResults.txt')
print(time.monotonic()-timeX)

enc.end()




