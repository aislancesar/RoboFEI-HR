PAN = 512 # Centralizes camera horizontally.
CALIB_TILT = 700 # Centralizes camera vertically for calibration.
RUN_TILT = 800 # Position to be used while running.

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
    if STATE == 0 and M[0] > len(main.img[0])/3 and M[0] < 2 * len(main.img[0])/3:
        print "Move Forward!"
        STATE = 1
        bkb.write_int(Mem, 'DECISION_ACTION_A', 8)
    elif STATE == 1:
        if M[0] == -1:
            print "Stop!"
            STATE = 4
            bkb.write_int(Mem, 'DECISION_ACTION_A', 0)
        elif M[0] < len(main.img[0])/3:
            print "Turn Left!"
            STATE = 2
            bkb.write_int(Mem, 'DECISION_ACTION_A', 2)
        elif M[0] > 2*len(main.img[0])/3:
            print "Turn Right!"
            STATE = 3
            bkb.write_int(Mem, 'DECISION_ACTION_A', 3)
    elif STATE == 2:
        if M[0] > len(main.img[0])/2:
            print "Walk Forward!"
            STATE = 1
            bkb.write_int(Mem, 'DECISION_ACTION_A', 8)
    elif STATE == 3:
        if M[0] < len(main.img[0])/2:
            print "Walk Forward!"
            STATE = 1
            bkb.write_int(Mem, 'DECISION_ACTION_A', 8)

    # Press 'q' to exit.
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

