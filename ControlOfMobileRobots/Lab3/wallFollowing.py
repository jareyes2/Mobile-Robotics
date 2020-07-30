import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import encoder_lib as my
import matplotlib.pyplot as plt
import math
import Movement as move
import Kinematics as kine

# Robot Physical Parameter
WHEEL_SEPERATION = 3.95
WHEEL_DIAMETER = 2.61
WHEEL_r = 2.61/2
DMID = 3.95/2

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b


def state(left, right):
    if left < 8:
        left = 'W'
    if right < 8:
        right = 'W'
    
    # Use both walls
    if left == 'W' and right == 'W':
        return 'LR'
    
    # Use right wall 
    if left != 'W' and right == 'W':
        return 'R'
    
    # Use left wall
    if left == 'W' and right != 'W':
        return 'L'
    
    # Use only forward motion
    if left != 'W' and right != 'W':
        return 'F'

    


# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)


''' Initialize Encoders '''
my.initEncoders()

''' Set to netural '''
my.setSpeedsPWM(1.5, 1.5)
pwmData, leftTotal, rightTotal = my.readCalibrateFile()

''' set cmin/cmax '''
cmax = max(leftTotal)
if max(rightTotal) < cmax:
    cmax = max(rightTotal)

cmin = min(leftTotal)
if min(rightTotal) > cmin:
    cmin = min(rightTotal)

cmax = cmax * (2*math.pi*WHEEL_r)
cmin = cmin * (2*math.pi*WHEEL_r)# Cmin / Cmax
cmax = 3
cmin = -3
# Proportional constant
kp = 1
rt = -5


while True:
    frontDistance = fSensor.get_distance()/25.4
    leftDistance = lSensor.get_distance()/25.4
    rightDistance = rSensor.get_distance()/25.4
    
    stateN = state(leftDistance,rightDistance)
    
    if(stateN == 'LR'):
        totalDistance = 4.375 + leftDistance + rightDistance
        rt_r = -(totalDistance - 4.375) / 2
        rt_l = rt_r
    else:
        rt_r = -5
        rt_l = -5
      
    yt, et, yt_r, et_r, yt_l, et_l = move.distancesToPIDParam(fSensor.get_distance(), rt, rSensor.get_distance(),rt_r, lSensor.get_distance(),rt_l)
    
    # PID functions
    ur   =  move.Ur(kp,rt,yt, cmax, cmin)
    ur_r =  move.Ur(kp,rt_r,yt_r, cmax, cmin)
    ur_l =  move.Ur(kp,rt_l,yt_l, cmax, cmin)
   
    if(et < 1 and et > -1):
        print("ONLY FRONT to front")
        my.setSpeedsIPS(ur,ur,leftTotal,rightTotal,pwmData)
    
    elif(stateN == 'LR'):
        print("In Hallway")
        my.setSpeedsIPS(ur, -ur_r + ur, leftTotal, rightTotal, pwmData)
        print("set speeds", ur, -ur_r + ur)
                  
    elif(stateN == 'L'):
        print("only left wall")
        my.setSpeedsIPS(-ur_l + ur, ur + ur_l, leftTotal, rightTotal, pwmData)
        print("set speeds", -ur_l + ur, ur+ ur_l)
    
    elif(stateN == 'R'):
        print("only right wall")
        my.setSpeedsIPS(ur + ur_r, -ur_r + ur, leftTotal, rightTotal, pwmData)
        print("set speeds", ur + ur_r, -ur_r + ur)
        
    elif(stateN == 'F'):
        print("only move forward")
        my.setSpeedsIPS(ur, ur, leftTotal, rightTotal, pwmData)
        print("set speeds", ur, ur)
    
    
     # When within error of front make turn
    if et <= .1 and et >= -.1 :
        print("reached front")
        my.setSpeedsIPS(0, 0, leftTotal, rightTotal, pwmData)
        
        if(stateN == 'LR'):
            move.Uturn(leftTotal,rightTotal,pwmData)
        
        elif(stateN == 'R'):
            move.LeftTurn(leftTotal,rightTotal,pwmData)
            
        elif(stateN == 'L' or stateN=='F'):
            move.RightTurn(leftTotal, rightTotal,pwmData)
        
        while stateN=='LR':
            frontDistance = fSensor.get_distance()/25.4
            leftDistance = lSensor.get_distance()/25.4
            rightDistance = rSensor.get_distance()/25.4
            
            if(rightDistance >= leftDistance):
                while (frontDistance < 10):
                    my.setSpeedsVW(0,.8,leftTotal,rightTotal, pwmData)
                    frontDistance = fSensor.get_distance()/25.4
                my.setSpeedsIPS(0, 0, leftTotal, rightTotal, pwmData)
                break
            
            elif(rightDistance < leftDistance):
                while (frontDistance < 10):
                    my.setSpeedsVW(0,-.8,leftTotal,rightTotal, pwmData)
                    frontDistance = fSensor.get_distance()/25.4
                my.setSpeedsIPS(0, 0, leftTotal, rightTotal, pwmData)
                break
     
        
# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()

