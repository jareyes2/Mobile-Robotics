import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import cv2 as cv
import time
from ThreadedWebcam import ThreadedWebcam
from UnthreadedWebcam import UnthreadedWebcam
import encoder_lib as my
import matplotlib.pyplot as plt
import math
import Movement as move
import numpy as np
import Kinematics as kine

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b


''' HSV colors '''
HSV_MIN = 0
HSV_MAX = 1
# Cylinder color
# Goal colors HSV
color1 = [(0,91,61),(9,132,176)]      # orange
color2 = [(42,106,55),(68,164,157)]   # Green
color3 = [(152,114,77),(173,185,194)] # pink
# Number of targets
numTargets =  3 


FPS_SMOOTHING = 0.9

# Window names
WINDOW1 = "Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

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

# Initialize the threaded camera
# You can run the unthreaded camera instead by changing the line below.
# Look for any differences in frame rate and latency.
camera = ThreadedWebcam() # UnthreadedWebcam()
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 40000
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
# Otherwise, one will be generated.
# These values WILL need to be adjusted for accurate and fast blob detection.
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

# PID parameters to face goal
cmax = 1.5
cmin = -1.5
# Proportional constant
kp = .01

# PID parameters for moving to goal
cmax_f = 5
cmin_f = -5
goal = 6
# Proportional constant
kp_f = 1

blobSizes = []
distances = []
goalFound = False


fps, prev = 0.0, 0.0
while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    frame = camera.read()
    Width=frame.shape[1]
    rt = Width/2 #desired distance 
    
    # Blob detection works better in the HSV color space 
    # (than the RGB color space) so the frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Create a mask using the given HSV range based on number of targets
    mask = cv.inRange(frame_hsv, color1[HSV_MIN], color1[HSV_MAX])
    if(numTargets > 1):
        mask2 = cv.inRange(frame_hsv, color2[HSV_MIN], color2[HSV_MAX])
        mask = cv.bitwise_or(mask,mask2)
    if(numTargets == 3):
        mask2 = cv.inRange(frame_hsv, color3[HSV_MIN], color3[HSV_MAX])
        mask = cv.bitwise_or(mask,mask2)
    
    # Run the SimpleBlobDetector on the mask.
    # The results are stored in a vector of 'KeyPoint' objects,
    # which describe the location and size of the blobs.
    keypoints = detector.detect(mask)
    
    # Get X coordinate of blob on camera
    if(len(keypoints) > 0):
        # X,Y points of blob
        pts = np.array(cv.KeyPoint_convert(keypoints))
        # Array of x points of blob
        pts = pts[:,0]
        # x point closest to middle
        x = pts[(np.abs(pts-rt)).argmin()]
    # No blobs detected spin around until one is found
    else:
        x = 0
    
    et = move.distanceToPIDParam(x, rt)
    #print("angular error",et)
    # PID functions
    w   =  move.Ur(kp,et, cmax, cmin)
    frontVelocity = 0
    #print("angular w", w)

    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Write text onto the frame
    #cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    #cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    
    # Display the frame
    cv.imshow(WINDOW1, mask)
    cv.imshow(WINDOW2, frame_with_keypoints)
    
    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

    
    if et < 15 and et > -15:
        # PID functions for front distance
        # print("distance", fSensor.get_distance()/25.4)
        error_front = move.distanceToPIDParam(fSensor.get_distance()/25.4, goal)
        # print("error front--", error_front)
        frontVelocity = move.Ur(kp_f,error_front,cmax_f,cmin_f)
        my.setSpeedsIPS(-frontVelocity,-frontVelocity,leftTotal,rightTotal,pwmData)
        blobSizes.append(keypoints[0].size)
        distances.append(fSensor.get_distance()/25.4)
        # print("move to goal")
        
    else:
        # Set angular speed
        w   =  move.Ur(kp, et, cmax, cmin)
        # Set angular speed
        my.setSpeedsVW(-frontVelocity,-w,leftTotal,rightTotal, pwmData)
        

camera.stop()