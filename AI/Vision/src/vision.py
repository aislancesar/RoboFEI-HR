PAN = 507 # Centralizes camera horizontally.
CALIB_TILT = 750 # Centralizes camera vertically for calibration.
RUN_TILT = 750 # Position to be used while running.

import sys
sys.path.append('../../Blackboard/src/')
from SharedMemory import SharedMemory 
import cv2
import os
import ctypes
import argparse

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
servo = Servo(PAN, CALIB_TILT)

# Calibrates the vision.
main.MainCalibration()
if args.step:
    main.StepCalibration()
if args.swerve:
    main.SwerveCalibration()

# Changes head position.
servo.writeWord(20, 30, RUN_TILT)

STATE = 0

# Creates a window to show the robots vision.
if args.show:
    cv2.namedWindow('Robot\'s Vision.')

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
    if STATE == 0:
        STATE = 1

    if STATE == 1:
        if M[0] == -1:
            STATE = 2
            
        print "Walk Forward!"
        bkb.write_float(Mem, 'VISION_OPP01_DIST', 0)
        bkb.write_float(Mem, 'VISION_OPP02_DIST', 0)
        bkb.write_float(Mem, 'VISION_OPP03_DIST', float (60 * (len(main.img[0])/2 - M[0]) / len(main.img[0])))
        bkb.write_int(Mem, 'DECISION_ACTION_A', 21)
    
    if STATE == 2:
        print "Stop!"
        bkb.write_int(Mem, 'DECISION_ACTION_A', 0)

    # Press 'q' to exit.
    # Press 'r' to Run Again
    k = cv2.waitKey(20) & 0xFF 
    if k == ord('q'):
        break
    if k == ord('r'):
        STATE = 0

cv2.destroyAllWindows()

