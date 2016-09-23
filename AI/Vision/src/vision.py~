PAN = 523 # Centralizes camera horizontally.
TILT = 750 # Centralizes camera vertically for calibration.

import sys
sys.path.append('../../Blackboard/src/')
from SharedMemory import SharedMemory 
import cv2
import os
import ctypes
import argparse
import time

from runningvision import *
from servo import Servo

# Creates a black board object.
bkb = SharedMemory()

mem_key = 100 # Sets the memory key.
Mem = bkb.shd_constructor(mem_key) # Creates a shared memory.

# Parsing of arguments on execution.
parser = argparse.ArgumentParser(description='Run Vision', epilog= 'Vision program used for the Humanoid Running Challenge.')
parser.add_argument('--step', '--s', action="store_true", help = 'Used for step challenge.')
parser.add_argument('--swerve', '--w', action="store_true", help = 'Used for swerve challenge.')
parser.add_argument('--show', '--i', action="store_true", help = 'Shows video while running.')

# Reads the arguments.
args = parser.parse_args()

# Sets some camera's flags.
os.system("v4l2-ctl -d /dev/video0 -c focus_auto=0 && v4l2-ctl -d /dev/video0 -c focus_absolute=0")

# Creates a vision object.
main = RunVision()

# Sets robot action to Stand Still.
bkb.write_int(Mem, 'DECISION_ACTION_A', 0)

# Initializes the head's control.
servo = Servo(PAN, TILT)

# Calibrates the vision.
main.MainCalibration()
if args.step:
    main.StepCalibration()
    # servo.writeWord(20, 30, 800)
if args.swerve:
    main.SwerveCalibration()

STATE = -1 # Initializes the FSM

# Creates a window to show the robots vision.
if args.show:
    cv2.namedWindow('Robot\'s Vision.')
    
S = (-1, -1, 0)
W = (-1, -1)
mS = 0

while True:
    # Capture a frame from the camera.
    main.capture()

    # Copies the frame.
    if args.show:
        res = cv2.repeat(main.img, 1, 1)
    else:
        res = None

    # Executes the functions.
    M = main.MainRunning(res)
    if args.step:
        S = main.StepRunning(res)
    if args.swerve:
        W = main.SwerveRunning(res)

    # Refreshes the window.
    if args.show:
        cv2.imshow('Robot\'s Vision.', cv2.resize(res, (int(len(res[0]) * main.scl), int(len(res) * main.scl))))

    # Prints the values.
    print "Main:", M,
    if args.step:
        print "- Step:", S,
    if args.swerve:
        print "- Swerve:", W,
    print

    # --- DECISION PROCESS ---
    # Initial State
    if STATE == 0:
        print "Initial State"
        STATE = 1 # Walking State
        X = 20    # Initial speed reduction
        servo.writeWord(20, 30, 750)
    
    # Walking State
    if STATE == 1:
        print "Walking State"
        if M[0] == -1: # If the track is over,
            STATE = 2  # go to Stop State.
        if S[0] != -1 and S[1] > len(main.img)/2: # If the robot perceives a near red stripe,
            STATE = 3  # go to Approaching State.
            servo.writeWord(20, 30, 830) # Lowers robot's head.
            mS = 0
#        if W[1] > len(main.img)/2 and W[0] < len(main.img[0])/2 and W[0] > len(main.img[0])/10:
#            STATE = 8
#        if W[1] > len(main.img)/2 and W[0] > len(main.img[0])/2 and W[0] < 9 * len(main.img[0])/10:
#            STATE = 9
        
        if X > 0:    # If there is any speed reduction,
            X -= 0.1 # lower it.
        
        # Forward Speed.
        bkb.write_float(Mem, 'VISION_OPP01_DIST', -X) 
        # Sideway Speed.
        bkb.write_float(Mem, 'VISION_OPP02_DIST', 0)
        # Turning Speed.
        bkb.write_float(Mem, 'VISION_OPP03_DIST', float (60 * (len(main.img[0])/2 - M[0]) / len(main.img[0])))
        # Move foward with correction.
        bkb.write_int(Mem, 'DECISION_ACTION_A', 21)
    
    # Stop State.
    if STATE == 2:
        print 'Stop State'
        # Stops the robot.
        bkb.write_int(Mem, 'DECISION_ACTION_A', 0)
    
    # Approach State.
    if STATE == 3:
        print 'Approach State'
        if mS > 3 * len(main.img)/5:
            STATE = 4
        if S[1] > 0:
            mS = 0.65 * mS + 0.35 * S[1]
            
        #bkb.write_float(Mem, 'VISION_OPP01_DIST', min(-10, max(-25, -12 * len(main.img) / mS)))
        #bkb.write_float(Mem, 'VISION_OPP02_DIST', 0)
        #bkb.write_float(Mem, 'VISION_OPP03_DIST', -S[2] * 30)
        #bkb.write_int(Mem, 'DECISION_ACTION_A', 21)
        bkb.write_int(Mem, 'DECISION_ACTION_A', 8)
    
    # Step Up State.
    if STATE == 4:
        print "Step Up!"
        bkb.write_int(Mem, 'DECISION_ACTION_A', 11)
        time.sleep(1)
        bkb.write_int(Mem, 'DECISION_ACTION_A', 22)
        time.sleep(10)
        bkb.write_int(Mem, 'DECISION_ACTION_A', 0)
        STATE = 5
    
    '''
    if STATE == 5:
        if S[0] != -1 and S[1] > len(main.img)/2:
            STATE = 6

        print "Go On!"
        bkb.write_float(Mem, 'VISION_OPP01_DIST', -10)
        bkb.write_float(Mem, 'VISION_OPP02_DIST', 0)
        bkb.write_float(Mem, 'VISION_OPP03_DIST', float (60 * (len(main.img[0])/2 - M[0]) / len(main.img[0])))
        bkb.write_int(Mem, 'DECISION_ACTION_A', 21)

    if STATE == 6:
        if S[0] == -1:
            STATE = 7

        print "Getting in Position!"
        bkb.write_float(Mem, 'VISION_OPP01_DIST', -20)
        bkb.write_float(Mem, 'VISION_OPP02_DIST', 0)
        bkb.write_float(Mem, 'VISION_OPP03_DIST', float (60 * (len(main.img[0])/2 - M[0]) / len(main.img[0])))
        bkb.write_int(Mem, 'DECISION_ACTION_A', 21)

    if STATE == 7:
        print "Step Down!"
        bkb.write_int(Mem, 'DECISION_ACTION_A', 23)
        STATE = 1

    if STATE == 8:
        if W[0] < len(main.img[0])/10:
            STATE = 1

        print "To the Right"
        bkb.write_int(Mem, 'DECISION_ACTION_A', 7)

    if STATE == 9:
        if W[0] > 9 * len(main.img[0])/10:
            STATE = 1

        print "To the Left"
        bkb.write_int(Mem, 'DECISION_ACTION_A', 6)
    '''
    # Press 'q' to exit.
    # Press 'r' to Run Again
    k = cv2.waitKey(20) & 0xFF 
    if k == ord('q'):
        break
    if k == ord('r'):
        STATE = 0
    if k == ord('t'):
        STATE = 2

cv2.destroyAllWindows()

