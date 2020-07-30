import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import cv2 as cv
import time
from ThreadedWebcam import ThreadedWebcam
import encoder_lib as my
import Movement as move
import numpy as np

# Trilateration functions
def A(x1,x2):
    return ((-2*x1) + (2*x2))

def B(y1,y2):
    return ((-2*y1) + (2*y2))

def C(r1,r2,x1,x2,y1,y2):
    return ((r1**2) - (r2**2) - (x1**2) + (x2**2) - (y1**2) + (y2**2))

def D(x2,x3):
    return ((-2*x2) + (2*x3))

def E(y2,y3):
    return ((-2*y2) + (2*y3))

def F(r2, r3, x2, x3, y2, y3):
    return ((r2**2) - (r3**2) - (x2**2) + (x3**2) -(y2**2) + (y3**2))

def x_pose(A,B,C,D,E,F):
    return ((C*E) - (F*B)) / ( (E*A) - (B*D))

def y_pose(A,B,C,D,E,F):
    return ((C*D)- (A*F)) / ((B*D) - (A*E))

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

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

# Initialize encoders
my.initEncoders()

# Set servos to netural
my.setSpeedsPWM(1.5, 1.5)
pwmData, leftTotal, rightTotal = my.readCalibrateFile()
blobSizes, distances = my.readBlobCalibrateFile()

# Tuple of colors with hsv min and max
HSV_MIN = 0
HSV_MAX = 1

# Goal colors HSV
color1 = [(0,91,61),(9,132,176)]      # orange
color2 = [(42,106,55),(68,164,157)]   # Green
color3 = [(152,114,77),(173,185,194)] # pink

# PID constants
cmax = .8
cmin = -.8
kp = .01

# Determine if goals have been encountered by robot
seenGoal1 = False
seenGoal2 = False
seenGoal3 = False

FPS_SMOOTHING = 0.9

# Window names
WINDOW1 = "Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Initialize the threaded camera
camera = ThreadedWebcam() 
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
# Filter by blob size, reduce blobs with area less than minArea pixels
params.filterByArea = True
params.minArea = 50000
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
if fs.isOpened():
    detector.read(fs.root())
else:
    print("WARNING: params file not found! Creating default file.")
    
    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
    detector.write(fs2)
    fs2.release()
    
fs.release()

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)

# Distances from goals
distanceC1 = 0
distanceC2 = 0
distanceC3 = 0
r1 = 0
r2 = 0
r3 = 0

while True:
    # Get a frame
    frame = camera.read()
    Width=frame.shape[1]
    rt = Width/2 #desired distance 
    
    # Blob detection works better in the HSV color space 
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Create a mask using the given HSV range
    if not seenGoal1:
        mask = cv.inRange(frame_hsv, color1[HSV_MIN], color1[HSV_MAX])
    elif not seenGoal2 and seenGoal1:
        mask = cv.inRange(frame_hsv, color2[HSV_MIN], color2[HSV_MAX])
    elif not seenGoal3 and seenGoal1 and seenGoal2:
        mask = cv.inRange(frame_hsv, color3[HSV_MIN], color3[HSV_MAX])
        
    # Run the SimpleBlobDetector on the mask and get 'KeyPoint' objects,
    keypoints = detector.detect(mask)
    
    # Get X coordinate of blob on camera
    if(len(keypoints) > 0):
        # X,Y points of blob
        pts = np.array(cv.KeyPoint_convert(keypoints))
        # Array of x points of blob
        pts = pts[:,0]
        # x point closest to middle
        idx = (np.abs(pts-rt)).argmin()
        x = pts[idx]
    # No blobs detected spin around until one is found
    else:
        x = 0
    
    # Error from goal using PID functions
    et = move.distanceToPIDParam(x, rt)
    w   =  move.Ur(kp,et, cmax, cmin)
    
    # Goal close to middle of frame
    if et < 8 and et > -8:
        
        # Stop motion of the robot to get distances 
        my.setSpeedsIPS(0,0,leftTotal,rightTotal,pwmData)
        
        # Checks which goal and finds distance to goal
        if not seenGoal1:
            i = 0
            while i < 30:
                distance1 = my.getDisanceFromBlob(keypoints[idx].size, blobSizes, distances)
                if  distance1 > 74:
                    distanceC1 += 65
                else:
                    distanceC1 += distance1
                i += 1
            r1 = distanceC1 / 30
            seenGoal1 = True
            my.setSpeedsVW(0,w,leftTotal,rightTotal, pwmData)
        
        elif not seenGoal2 and seenGoal1 and not seenGoal3:
            i = 0
            while i < 30:
                distance2 = my.getDisanceFromBlob(keypoints[idx].size, blobSizes, distances)
                if distance2 > 74:
                    distanceC2 += 65
                else:
                    distanceC2 += distance2
                i += 1
            r2 = distanceC2 / 30
            seenGoal2 = True
            my.setSpeedsVW(0,w,leftTotal,rightTotal, pwmData)
            
        elif not seenGoal3 and seenGoal1 and seenGoal2:
            i = 0
            while i < 30:
                distance3 = my.getDisanceFromBlob(keypoints[idx].size, blobSizes, distances)
                if distance3 > 74:
                    distanceC3 += 65
                else:
                    distanceC3 += distance3
                i += 1
            r3 = distanceC3 / 30
            seenGoal3 = True
            camera.stop()
            break 

    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
   
    # Display camera the frames
    cv.imshow(WINDOW1, mask)
    cv.imshow(WINDOW2, frame_with_keypoints)
    
    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break
    
    #based on PID set rotational speed
    my.setSpeedsVW(0,-w,leftTotal,rightTotal, pwmData)

        

camera.stop()

# Locations of goals in maze
##color1 = (4,4)
##color2 = (3,53)
##color3 = (58,56)

# External locations
color1 = (-4,3)
color2 = (4,66)
color3 = (67,57.5)


x1, y1 = color1
x2, y2 = color2
x3, y3 = color3

# Trilateration equaltions
a_ = A(x1,x2)
b_ = B(y1,y2)
c_ = C(r1,r2,x1,x2,y1,y2)
d_ = D(x2,x3)
e_ = E(y2,y3)
f_ = F(r2, r3, x2, x3, y2, y3)
print()

# x,y loction of the robot
x_curr = x_pose(a_,b_,c_,d_,e_,f_)
y_curr = y_pose(a_,b_,c_,d_,e_,f_)

print("Distances away from goal1, goal2, goal3: ",r1,r2,r3)
print("Robot (x,y):" ,x_curr,y_curr)
