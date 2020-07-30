#task 2 Distance
#This task is making the robot go a certain distance inputted by the user in a number of seconds
import time
import math
import Kinematics as km
import encoder_lib as en

# Robot physical parameters
WHEEL_SEPERATION = 3.95
WHEEL_DIAMETER = 2.61
DMID = WHEEL_SEPERATION/2
WHEEL_r = WHEEL_DIAMETER/2

# Check if linear speed requested is possible by robot
def checkInputLinearIPS(ips,leftData,rightData):
    
    # Get true min/max values possible convert from rotations per second to inches per second
    maxL = max(leftData)  * (2*math.pi*WHEEL_r)
    maxR = max(rightData) * (2*math.pi*WHEEL_r)
    minL = min(leftData)  * (2*math.pi*WHEEL_r)
    minR = min(rightData) * (2*math.pi*WHEEL_r)
    
    # Check if inches/second (distance/time) requested is not possible robot 
    if ips > maxL or ips > maxR or ips < minL or ips < minR:
        print('Excedes revolutions per second that the wheels can travel')
        return False
    
    # If inches per second X/Y is possible return value
    return ips

# Checks if input based in rotations rps exceeds max possible by either servos and prints message
def checkInputRPS(rps, lData, rData):
    # Get max/min RPS possible by physical robot
    maxR = max(rData)
    maxL = max(lData)
    minR = min(rData)
    minL = min(lData)
    
    # If rps requested is exceded print message
    if rps > maxR or rps > maxL or rps < minL or rps < minR:
        print("Excedes revolutions per second that the wheels can travel")
        return False
    
    return True

# Linear movement given distance and ips inches per second 
def linearMovement(ips, distance, leftTotal, rightTotal, pwmData):
    # Check if inches per second are possible
    if not checkInputLinearIPS(ips, leftTotal, rightTotal):
        print("Inches per second excedes phyical parameters")
        
    # Get counts from encoders at start 
    ticksL,ticksR = en.getCounts()
    # Set forward speeds and begin linear movement
    timeStart = time.monotonic()
    en.setSpeedsIPS(ips, ips, leftTotal, rightTotal, pwmData)
    while True:
        # Calculate distance of wheels traveled based on encoder counts (ticks)
        distanceL, distanceR = km.DistanceWheels(ticksL,ticksR)
        
        # Break loop when complete distance is traveled
        if(distanceR >= abs(distance) or distanceL >= abs(distance)):
            en.setSpeedsPWM(1.5, 1.5)
            #print("Distance Left Right: ", distanceL, ' ',distanceR)
            #print('Time taken to complete task: ', time.monotonic() - timeStart)
            break
    return

# Orient robot based on w = radians/time to complete angle of rotation
# Turn angle is in radians 
def orientationMovement(w, turnAngle, leftTotal,rightTotal,pwmData):
    # Check if inches per second are possible
    if not checkInputLinearIPS(w, leftTotal, rightTotal):
        print("Inches per second excedes phyical parameters")

    # Calculate arch length (distance) to travel given turnAngle and motion is opientation (R = DMID)
    distance = turnAngle*DMID
    #print('Distance each wheel travels in inches: ', distance)
    
    # Sets speeds to angular velocity w, stops based on distance of wheels traveled calculated 
    ticksL,ticksR = en.getCounts()
    en.setSpeedsVW(0,w,leftTotal,rightTotal, pwmData)
    timeStart = time.monotonic()
    while True:
        # Calculate distance of wheels traveled based on encoder counts (ticks)
        distanceL, distanceR = km.DistanceWheels(ticksL,ticksR)
        
        # Break loop when complete distance is traveled
        if(distanceR >= abs(distance) or distanceL >= abs(distance)):
            en.setSpeedsPWM(1.5, 1.5)
            #print("Distance Left Right: ", distanceL, ' ',distanceR)
            #print('Time taken to complete task: ', time.monotonic() - timeStart)
            break
    return


# Functon to use for slowing doen before turn
def stopForTurn (t=.5):
    en.setSpeedsPWM(1.5,1.5)
    timeStartWaste = time.monotonic()
    while(time.monotonic()-timeStartWaste < t):
        continue
    return

'''PID motion control '''
# Fsat function
def Fsat(ut, cmax, cmin): 
    if ut > cmax:
        return cmax
    elif ut >= cmin and ut <= cmax:
        return ut
    elif ut < cmin:
        return cmin
    
# Ur   
def Ur(kp, et, cmax, cmin):
    return Fsat((kp * et), cmax, cmin)

# Sensor data to PID
def distanceToPIDParam(sensor, rt):
    # Convert to inches and get distance to goal and distance away from obstacle for front right left senors
    yt = sensor
    et = rt - yt
    
    return et

def distancesToPIDParamFLR(fSensor, rt, lSensor,rt_l, rSensor,rt_r):
    # Convert to inches and get distance to goal and distance away from obstacle for front right left senors
    yt = - fSensor/25.4
    et = rt - yt

    yt_r = -rSensor/25.4
    et_r = rt_r - yt_r
    
    yt_l = - lSensor/25.4
    et_l = rt_l - yt_l
    
    return yt, et, yt_r, et_r, yt_l, et_l

# Display for debugging
def Display(y,e,yr,er,yl,el):
    print("yt: ",y)
    print("yt_r: ",yr)
    print("yt_l: ",yl)
    print()
    print("e: ",e)
    print("e_r: ",er)
    print("e_l: ",el)
    
def Uturn(leftTotal,rightTotal,pwmData):
    stopForTurn(.8)
    orientationMovement(.8, math.pi, leftTotal,rightTotal,pwmData) 
    en.onUturn()

def RightTurn(leftTotal,rightTotal,pwmData):
    stopForTurn(.8)
    orientationMovement(1, math.pi/2, leftTotal,rightTotal,pwmData) 
    en.onRTurn() #global counter for right turns
    stopForTurn(.8)
  
def LeftTurn(leftTotal,rightTotal,pwmData):
    stopForTurn(.8)
    orientationMovement(-1, math.pi/2, leftTotal,rightTotal,pwmData) # left turn
    en.onLTurn() #global counter for left turns
    stopForTurn(.8)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    


