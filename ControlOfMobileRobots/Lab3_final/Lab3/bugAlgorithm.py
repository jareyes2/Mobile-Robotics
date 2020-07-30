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

# state variables
state          = "starting"
angular_error  = None
blob_size      = 0
front_distance = None
left_distance  = None

# Window names
WINDOW1 = "Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"


# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

# HSV target parameters
HSV_MIN = 0
HSV_MAX = 1
# Cylinder color
# Goal colors HSV
color3 = [(0,91,61),(9,132,176)]      # orange
color2 = [(42,106,55),(68,164,157)]   # Green
color1 = [(152,114,77),(173,185,194)] # pink

goalColor = color1
    
def currentState():
    
    global state
    global angular_error
    global blob_size    
    global front_distance 
    global left_distance

    # starting state
    if state == "starting":
        state = "face goal"
        return

    if state == "face goal":
        # If the angular error is in this range the robot is facing the goal
        if angular_error < 17 and angular_error > -17:
            state = "check blob"
            currentState()
            return
        # Continue looking for goal
        else:
            state = "face goal"
            return
        
    if state == "check blob":
        # If the blob size is greater 500 then we have reached the goal
        print("checking Blob", blob_size)
              
        if blob_size > 400:
            state = "done"
            return
        # If not at goal and no obstacle 
        elif front_distance > 5:
            state = "motion to goal"
            return
        
        else:
            state = "turn"
            #state = "wall following"
            return
        
    if state == "motion to goal":
        # if no obstacle go to check blob
        if front_distance > 5:  #10 cm = 3.94 inches
            state = "check blob"
            currentState()
            return
        # if wall distance is less than 
        elif front_distance <= 6:
            state = "turn"
            return
    
    if state == "wall following":
        if front_distance > 8 and angular_error < 17 and angular_error > -17:
            state = "face goal"
            return
        else:
            state = "wall following"
            return            
            
                  
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

# initialize encoders
my.initEncoders()

# Set robot speeds to nuetral 
my.setSpeedsPWM(1.5, 1.5)
pwmData, leftTotal, rightTotal = my.readCalibrateFile()

# Initialize the threaded camera
camera = ThreadedWebcam() 
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
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

# PID parameters to face goal
cmax = 1.5
cmin = -1.5
# Proportional constant
kp = .01

# PID parameters for moving to goal
cmax_f = 4
cmin_f = -4
goal = 5
kp_f = 1
goalWall = 5


fps, prev = 0.0, 0.0
while True:
    # Get a frame
    frame = camera.read()
    Width = frame.shape[1]
    rt_targetInframe  = Width/2  #desired distance is to have goal centered in frame
    
    # Frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # Create a mask using the given HSV range based on number of targets
    mask = cv.inRange(frame_hsv, goalColor[HSV_MIN], goalColor[HSV_MAX])
    # Run the SimpleBlobDetector on the mask get keypoints object
    keypoints = detector.detect(mask)
    
    # Get X coordinate of blob on camera
    if(len(keypoints) > 0):
        # X,Y points of blob
        pts = np.array(cv.KeyPoint_convert(keypoints))
        # Array of x points of blob
        pts = pts[:,0]
        # x equals point closest to middle
        indx = (np.abs(pts-rt_targetInframe)).argmin()
        x = pts[indx]
        blob_size = keypoints[indx].size
        
    # No blobs detected spin around until one is found
    else:
        x = 0
        
    # Distance of blob from centered location
    angular_error = move.distanceToPIDParam(x, rt_targetInframe)
    # Get distance sensor data
    front_distance = fSensor.get_distance()/25.4
    left_distance  = lSensor.get_distance()/25.4
    
    # Get current state of robot 
    currentState()
    
    # Determine motion based on state
    if state == "starting":
        print(state)
        
    elif state == "face goal":
        print(state)
        angular_error = move.distanceToPIDParam(x, rt_targetInframe)
        # PID functions
        w   =  move.Ur(kp, angular_error, cmax, cmin)
        my.setSpeedsVW(0,-w,leftTotal,rightTotal, pwmData)
        
    elif state == "motion to goal":
        # Motion to go
        print(state)
        # PID functions
        w   =  move.Ur(kp, angular_error, cmax, cmin)
        frontVelocity = 0
        #print("angular w", w)
        
        if angular_error < 17 and angular_error > -17:
            print("facing goal")
            # PID functions for front distance
            error_front = move.distanceToPIDParam(fSensor.get_distance()/25.4, goal)
            # print("error front--", error_front)
            frontVelocity = move.Ur(kp_f,error_front,cmax_f,cmin_f)
            my.setSpeedsIPS(-frontVelocity,-frontVelocity,leftTotal,rightTotal,pwmData)
            
        else:
            # Set angular speed
            w   =  move.Ur(kp, angular_error, cmax, cmin)
            my.setSpeedsVW(-frontVelocity,-w,leftTotal,rightTotal, pwmData)
            
    elif state == "wall following":
        print(state)
        if(fSensor.get_distance()/25.4 < 5):
            state = "turn"
            continue
        
        error_front_wf = move.distanceToPIDParam(fSensor.get_distance()/25.4, 4)
        frontVelocity_wf = move.Ur(1,error_front_wf,3,-3)
        print("et", frontVelocity_wf)
        
        error_wall = move.distanceToPIDParam(lSensor.get_distance()/25.4, goalWall)
        sideVelocity = move.Ur(kp_f,error_wall, cmax, cmin)
        my.setSpeedsIPS(-frontVelocity_wf - -sideVelocity, -frontVelocity_wf + -sideVelocity, leftTotal, rightTotal, pwmData)
        state = "wall following"

    elif state == "done":
        print(state)
        my.setSpeedsIPS(0,0,leftTotal,rightTotal, pwmData)
        state = "check blob"
        
    elif state == "turn":
        print(state)
        my.setSpeedsIPS(0,0,leftTotal,rightTotal, pwmData)
        move.RightTurn(leftTotal,rightTotal,pwmData)
        while (fSensor.get_distance()/25.4 < 5):
            my.setSpeedsVW(0,.8,leftTotal,rightTotal, pwmData)
        while (lSensor.get_distance()/25.4 < 4):
            my.setSpeedsVW(0,.8,leftTotal,rightTotal, pwmData)
            
        state = "wall following"
                

    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Display the frame
    cv.imshow(WINDOW1, mask)
    cv.imshow(WINDOW2, frame_with_keypoints)
    
    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

    
    
camera.stop()
my.end()


