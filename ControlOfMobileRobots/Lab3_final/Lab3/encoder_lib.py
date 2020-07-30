#encoder_lib.py
#This program contains all of the encoder and motor control functions

import time
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/ 
import signal # README: https://raspberry-projects.com/pi/programming-in-c/signal-handling/signal-function
import Adafruit_PCA9685
import math
import numpy as np
from scipy import interpolate
import Kinematics as kin


''' Constants: Physical Parameters, Encoders, Servos, GPIO'''
# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# Global encoder Counters
LCOUNTER = 0 
RCOUNTER = 0

# Robot Physical Parameters
WHEEL_SEPERATION = 3.95
WHEEL_DIAMETER = 2.61
WHEEL_r = 2.61/2

UTURN = 0
LTURN = 0
RTURN = 0


''' B1 Servo: handle interrupts and encoders and ctrlC'''
# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global LCOUNTER
    LCOUNTER += 1
   
# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global RCOUNTER
    RCOUNTER += 1
    
def onUturn():
    print('U Turn')
    global UTURN
    UTURN += 1
   
# This function is called when the right encoder detects a rising edge signal.
def onRTurn():
    print('Right Turn')
    global RTURN
    RTURN += 1

# This function is called when the right encoder detects a rising edge signal.
def onLTurn():
    print('Left Turn')
    global LTURN
    LTURN += 1
      
# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    onTerminate()
    print()
    GPIO.cleanup()
    # Stop the servos
    exit()

def end():
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    onTerminate()
    print()
    GPIO.cleanup()
    # Stop the servos
    exit()
  
  
# Print values for turns if exiting with intterupt 
def onTerminate():
    # Get number of turns performed by robot with global variables
    global UTURN
    global RTURN
    global LTURN
    
    # Get numnber of encoder ticks to determine distance travelled
    wheelCounts = getCounts()
    wheelL = wheelCounts[0]
    wheelR = wheelCounts[1]

    # Get distance each each travelled
    wheelLDistance = wheelL * (2*math.pi*WHEEL_r)
    wheelRDistance = wheelR * (2*math.pi*WHEEL_r)
    
    # Print Distances and turns
    print('Total Distance Traveled in inches for left wheel: ', wheelLDistance)
    print('Total Distance Traveled in inches for right wheel: ', wheelRDistance)
    print('Robot total distance: ', (wheelLDistance + wheelRDistance)/2)
    print('Number of Uturns: ', UTURN)
    print('Number of Lturns: ', LTURN)
    print('Numebr of RTurns: ', RTURN)
    


    
       
    
       
''' Encoder Functions: Get counts, speeds, reset counts'''
# Should reset the tick count (number of holes counted) to zero ie.encoder counts = 0
def resetCounts():
    global LCOUNTER
    global RCOUNTER
    LCOUNTER = 0
    RCOUNTER = 0
    return (LCOUNTER, RCOUNTER)

# Should return left and right tick counts since last call to resetCounters or start
def getCounts():
    global LCOUNTER
    global RCOUNTER
    return (LCOUNTER,RCOUNTER)

# This function will initialize encoders 
# Set the pin numbering scheme to the numbering shown on the robot itself
def initEncoders():
    GPIO.setmode(GPIO.BCM)

    # Set encoder pins as input
    # Also enable pull-up resistors on the encoder pins
    # This ensures a clean 0V and 3.3V is always outputted from the encoders.
    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Attach a rising edge interrupt to the encoder pins
    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# Should return the instantaneous left and right wheel speeds in rps (Revolutions Per Seconds) 
# 1 revoution = 32 encoder interrupts 
def getSpeeds(t=1):
    # Save current encoder counts for left and right wheel
    rightCurr = RCOUNTER
    leftCurr = LCOUNTER
    
    # save time at start and stay in loop for t amount of time 
    timeStart = time.monotonic()
    while True:
        if(time.monotonic() - timeStart > t):
            timeEnd = time.monotonic()
            break
        rTicks = RCOUNTER - rightCurr
        lTicks = LCOUNTER - leftCurr
    
    totalTime = timeEnd-timeStart
    #return number revolutions per time interval t (32 = 1 revolution)
    return ((lTicks/32)/totalTime,(rTicks/32)/totalTime)   

    
    
    
    
    
    
    
    
''' Motor Control: setting PWM, RPS, IPS, VW, velocities for wheels '''
# Should set the speed of the motors in PWM numbers 1.3 - 1.7 
def setSpeedsPWM(pwmLeft, pwmRight):
    #print("setting speeds pwm to: L= ",pwmLeft, " R= ", pwmRight)
    pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(pwmRight / 20 * 4096))

# This function sets speeds in revolutions per second
# Converts rps to pwm using results from calibrateSpeed function by mapping the rps to pwm
def setSpeedsRPS(rpsLeft, rpsRight, lSpeeds, rSpeeds, pwmData):
    
    # get difference of speed in list from actual speed wanted and save in list for left and right wheels
    # Closest to zero is the pwm choosen (executed for left and right)
    lSpeeds = np.asarray(lSpeeds)
    idxL = (np.abs(lSpeeds-rpsLeft)).argmin()
    finalLeft = pwmData[idxL]
    
    rSpeeds = np.asarray(rSpeeds)
    idxR = (np.abs(rSpeeds-rpsRight)).argmin()
    finalRight = pwmData[idxR]
    
    # Set pwm for equivalent rpm 
    setSpeedsPWM(finalLeft,finalRight)
    
    # return pwm used 
    return (finalLeft, finalRight)
    
# should set the speed of the motors best on inches per second input
# converts IPS to rps to pwm
def setSpeedsIPS(ipsLeft, ipsRight,lSpeeds,rSpeeds,pwmData):
    # one revolution of the wheel travels full circumferance 2*pi*r
    rpsL = ipsLeft/(2*math.pi*WHEEL_r)
    #print('ips: ', ipsLeft, 'rpsL: ', rpsL)
    rpsR = ipsRight/(2*math.pi*WHEEL_r)
    #print('ips: ', ipsRight, 'rpsR: ', rpsR)

    # call function to set speeds based on RPS 
    finalPWM = setSpeedsRPS(rpsL,rpsR,lSpeeds,rSpeeds,pwmData)

    # Return what pwms were used (finalPWM is tuple of PWM including Left and Right)
    return (finalPWM[0],finalPWM[1])

# Set speed given linear and angular velocity
# Should set the speed of the robot, so that the robot will move with a linear speed v and angular velocity w
def setSpeedsVW(v,w,leftSpeeds,rightSpeeds, pwmData):
    # R is radius of circular path
    if w == 0:
        R = 10000000000
    else:
        R = v/w
    
    # Calculate velocity for left and right wheel
    v_l = kin.WheelVelocityL(w,R)
    v_r = kin.WheelVelocityR(w,R)
    
    # Set pwm with IPS function 
    setSpeedsIPS(v_l,v_r,leftSpeeds,rightSpeeds, pwmData)

    # Return left and right wheel velocities calculated for each wheel
    return (v_l, v_r)

# Read calibration file and return the results as lists 
def readCalibrateFile(filename = 'CalibrateResults.txt'):
    #Store results in list 
    leftResults = []
    rightResults = []
    PWMintervals = []
    
    #open calibration file
    with open(filename, 'r') as f:
        for x in f:
            y = x.split()
            y = [float(i) for i in y]
            PWMintervals.append(y[0])
            leftResults.append(y[1])
            rightResults.append(y[2])
    
    #return results in list
    return PWMintervals, leftResults, rightResults


# Read blob calibration file and return the results as lists 
def readBlobCalibrateFile(filename = 'BlobCalibration.txt'):
    #Store results in list 
    blobSizes = []
    distances = []
    
    #open calibration file
    with open(filename, 'r') as f:
        for x in f:
            y = x.split()
            y = [float(i) for i in y]
            blobSizes.append(y[0])
            distances.append(y[1])
        
    #return results in list
    return blobSizes, distances

# Wprking
def getDisanceFromBlob(currBlob, blobSizes, distances):
    
    blobSizes = np.asarray(blobSizes)
    idx = (np.abs(blobSizes-currBlob)).argmin()
    finalDistance = distances[idx]
    
    return finalDistance

# use the encoder functions to create a mapping from the servos input (micro seconds)
# To the servo output (wheel speed in revolutions per second)
# Measure the speed of the wheels for different input values and store the results for both wheels    
def calibrateSpeeds(timeCal=1, filename="CalibrateResults.txt"):
    # Lists to save ave speeds for both wheels and respective pwm
    aveLeftSpeed = []
    aveRightSpeed = []
    currPWM = []
    
    #temporary to get average speed
    speedR = 0
    speedL = 0
    
    #set to neutral pwm
    setSpeedsPWM(1.5, 1.5)
    
    # start pwm at 1.3 and increment by .01 
    pwm = 130
    while True:
        currPWM.append(pwm/100)

        # pwm in range 1.3 - 1.7
        setSpeedsPWM(pwm/100, pwm/100)
        
        # wait a short period of time after setting speeds to ensure wheels are spinning at the specified PWM
        timeStartWaste = time.monotonic()
        while(time.monotonic()-timeStartWaste < .5):
            continue
            
        # run each pwm 3 times to account for discrepancies 
        for x in range(3):
            temp = getSpeeds(timeCal)
            speedL += temp[0]
            speedR += temp[1]
        
        # Assign directionality to spin direction of left and right wheels
        if(pwm < 150):
            speedL = -1*speedL
        elif(pwm > 150):
            speedR = -1*speedR
        
        # Append ave speed @ specified pwm to list
        aveLeftSpeed.append(speedL/3)
        aveRightSpeed.append(speedR/3)
    
        # Increment pwm
        pwm += 1
        speedL = 0
        speedR = 0
        
        # Once calibration complete break loop and set robot to neutral 
        if pwm > 170:
            setSpeedsPWM(1.5, 1.5)
            break
    
    # Use linear interpolation to up sample giving .001 resolution from 0.01 resolution of PWM
    newPWM = np.arange(1.3,1.701,0.001)
    f_L = interpolate.interp1d(currPWM, aveLeftSpeed, kind='linear')
    f_R = interpolate.interp1d(currPWM, aveRightSpeed, kind='linear')
    newL = f_L(newPWM)
    newR = f_R(newPWM)
    
    # create file to save calibration results to
    file = open(filename,"w")
    count = 0
    for x in newPWM:
        #L is the line to write in outgoing file consisting of pwm and speeds
        L = str(x) + " " + str(newL[count]) + " " + str(newR[count]) + "\n"
        file.write(L)
        count+=1
    file.close()

    # Return list of ave speeds empirical (not interpolated)
    return currPWM, aveLeftSpeed,aveRightSpeed


# Calibration using 0.005 resolution  
def calibrateSpeeds005(timeCal=1, filename="CalibrateResults.txt"):
    # Lists to save ave speeds for both wheels and respective pwm
    aveLeftSpeed = []
    aveRightSpeed = []
    currPWM = []
    
    #temporary to get average speed
    speedR = 0
    speedL = 0
    
    #set to neutral pwm
    setSpeedsPWM(1.5, 1.5)
    
    # start pwm at 1.3 and increment by .01 
    pwm = 1300
    while True:
        currPWM.append(pwm/1000)

        # pwm in range 1.3 - 1.7
        setSpeedsPWM(pwm/1000, pwm/1000)
        
        # wait a short period of time after setting speeds to ensure wheels are spinning at the specified PWM
        timeStartWaste = time.monotonic()
        while(time.monotonic()-timeStartWaste < .3):
            continue
            
        # run each pwm 2 times to account for discrepancies 
        for x in range(2):
            temp = getSpeeds(timeCal)
            speedL += temp[0]
            speedR += temp[1]
        
        # Assign directionality to spin direction of left and right wheels
        if(pwm < 1500):
            speedL = -1*speedL
        elif(pwm > 1500):
            speedR = -1*speedR
        
        # append ave speed @ specified pwm to list
        aveLeftSpeed.append(speedL/2)
        aveRightSpeed.append(speedR/2)
    
        # Increment pwm
        pwm += 5
        speedL = 0
        speedR = 0
        
        # Once calibration complete break loop and set robot to neutral 
        if pwm > 1700:
            setSpeedsPWM(1.5, 1.5)
            break
    
    #get 0.001 interpolated results
    newPWM = np.arange(1.3,1.701,0.001)
    f1 = interpolate.interp1d(currPWM, aveLeftSpeed, kind='linear')
    f2 = interpolate.interp1d(currPWM, aveRightSpeed, kind='linear')
    newL = f1(newPWM)
    newR = f2(newPWM)
    
    # create file to save calibration results to
    file = open(filename,"w")
    count = 0
    for x in newPWM:
        #L is the line to write in outgoing file consisting of pwm and speeds
        L = str(x) + " " + str(newL[count]) + " " + str(newR[count]) + "\n"
        file.write(L)
        count+=1
    file.close()

    # Return lists of results seen (not interpolated)
    return currPWM, aveLeftSpeed,aveRightSpeed



''' Inititialize Settings '''
# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)




