# This file is used for kinematic calcuations
import math
import encoder_lib as en

# Robot Physical Parameters
WHEEL_SEPERATION = 3.95
DMID = 3.95/2
WHEEL_DIAMETER = 2.61
WHEEL_r = 2.61/2
WHEEL_C = 2 * math.pi * WHEEL_r

# Returns wheel circumference
def WheelCircumference():
    c = 2 * math.pi * WHEEL_r
    return c

# Returns angular velocity of 
def Angular_w(vl,vr):
    return (vl - vr)/ (2 * DMID)

# Returns radius of ciruclar motion robot moves when wheel velocities are given
def Angular_R(vl,vr):
    return DMID * ((vr + vl) / (vl-vr))

# Returns velocity of wheel when angular velocity and radius are given
def WheelVelocityL(w,R):
    return  w * (R+DMID)
    
# Returns velocity of wheel when angular velocity and radius are given
def WheelVelocityR(w,R):
    return w * (R-DMID)

# Returns robots velocity with parameters w and R
def VelocityRobot_wr(w,R):
    return w * R

# Returns robots velocity when wheel velocities are given
def VelocityRobot_vv(vl, vr):
    return (vr + vl) / 2

# Return distance traveled by each wheel in tuple
def TotalDistanceWheels():
    ticksL, ticksR = enc.getCounts()
    distanceL = ticksL * (WHEEL_C/32)
    distanceR = ticksR * (WHEEL_C/32)
    return (distanceL, distanceR)

# Return distance traveled by each wheel from a starting point of encoder ticks
def DistanceWheels(ticksL,ticksR):
    # Get current number of encoder ticks
    ticksEndL, ticksEndR = en.getCounts()
   
    # Calculate distance wheels have traveled since encoder ticks given
    distanceL = (ticksEndL - ticksL) * (WHEEL_C/32)
    distanceR = (ticksEndR - ticksR) * (WHEEL_C/32)
    return (distanceL, distanceR)

# Calculate arc lecnth given thera and radius
def ArcLength(theta, R):
    return theta * R

# Caluculate the angle theta given angular velocity and time
def Theta(w,time):
    return w * time



    