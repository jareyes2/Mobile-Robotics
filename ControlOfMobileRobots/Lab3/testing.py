'''
Robot can do motion-to-goal and wall-following with 1 obstacle and internal goal (5 points)
o Robot can do motion-to-goal and wall-following with 2 obstacles and internal goal (5 points)
o Robot can do motion-to-goal and wall-following with 1 obstacle and one of the external goals (5 points)
o Robot can do motion-to-goal and wall-following with 2 obstacles and one of the external goals (5 points)
o Robot hits obstacle (-3 points each time)
o Robot fails to switch back to motion-to-goal (-4 points each time) o Full task is not completed under 2 minutes (-5 points)


obstacles will be no closer than 12 inches from each other so the robot has enough space to move between them


 to move the robot towards one of the goals while avoiding obstacles

The robot needs to perform face-goal and motion-to-goal,
but if an obstacle is detected immediately in front of the robot -- the distance on the front sensor is less than 10 cm
switch to wall-following

Wall-following will continue until the robot has line-of- sight of the goal AND there is no obstacle immediately in front of the robot



'''

# State Machine 
faceGoal            = False
motionToGoal        = False
wallFollowing       = False

obstacleDetected    = False
seesGoal            = False #goal within 10 cm (4 inches)
currState           = 0



def state(currState, faceGoal, obstacleDetected):
    start = 0
    obstructed = 1
    toGoal = 
    
    if currState == start:
        print(0)
        
 
    print(start, faceGoal, obstacleDetected)