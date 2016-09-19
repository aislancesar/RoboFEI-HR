import cv2
import numpy as np
from math import *

# This class is the vision implementation used for the Humanoid Robot Running Competition.
class RunVision:
# --- Class initializer ---------------------------------------------------------------------------
    
    def __init__ (self, blur=51, show=True, radius=10, threshold=10, scale=1, video=0):
        # Vars used through the entire application.
        # Limits for color segmentation of the main running task.
        self.main_lower = np.array([255, 255, 255])
        self.main_upper = np.array([0, 0, 0])
        # Limits for color segmentation of the step challenge.
        self.step_lower = np.array([255, 255, 255])
        self.step_upper = np.array([0, 0, 0])
        # Limits for color segmentation of the swerve challenge.
        self.swerve_lower = np.array([255, 255, 255])
        self.swerve_upper = np.array([0, 0, 0])
        # Blur factor used to clear image noise.
        self.blur = blur
        # Shows frames while execution.
        self.show = show
        # Holds captured frames.
        self.img = None
        # Holds blurred frames.
        self.imgblur = None
        # Holds hsv frames.
        self.hsv = None

        # Vars used for segmentation's calibration.
        # Saves the mouse's position to draw it on screen.
        self.pos = (None, None)
        # Radius used to select the colors for segmentation.
        self.rad = radius
        # Threshold from the mean color for segmentation.
        self.thrs = threshold
        # Screen scalation factor.
        self.scl = scale

        # Load Camera
        self.cap = cv2.VideoCapture(video)
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

# --- This method resets the segmentions limits ---------------------------------------------------
    
    def reset(self, x):
        if x == 0:
            self.main_lower = np.array([255, 255, 255])
            self.main_upper = np.array([0, 0, 0])
        elif x == 1:
            self.step_lower = np.array([255, 255, 255])
            self.step_upper = np.array([0, 0, 0])
        elif x == 2:
            self.swerve_lower = np.array([255, 255, 255])
            self.swerve_upper = np.array([0, 0, 0])

# --- Function that captures frames and applies segmentation --------------------------------------
    
    def capture(self):
        try:
            # self.img = cv2.imread('zxcvb.jpg') # Get image from camera
            _, self.img = self.cap.read()
            self.imgblur = cv2.medianBlur(self.img, self.blur) # Blurs image
            self.hsv = cv2.cvtColor(self.imgblur, cv2.COLOR_BGR2HSV) # Convert to HSV
        except:
            print "Error on frame initialization."

# --- Mouse Events --------------------------------------------------------------------------------

    def Segment(self, event, x, y, flags, param):
        # Refreshs mouse's pointer position.
        if event == cv2.EVENT_MOUSEMOVE:
            self.pos = (int(x / self.scl), int(y / self.scl))

        # Executed when a click happens
        if event == cv2.EVENT_LBUTTONDOWN:
            # Gets x and y from scaled image.
            x = int(x / self.scl)
            y = int(y / self.scl)

            # Vector with the weights.
            P = np.array([])
            # Vector with the weighed sum of the HSV values of the point.
            hM = np.array([])
            sM = np.array([])
            vM = np.array([])

            # Iterates through all points around the click to compute their weighs.
            for i in range(self.rad):
                for j in range(self.rad):
                    try:
                        # Relative x and y positions.
                        ai = x + i - int(self.rad/2)
                        aj = y + j - int(self.rad/2)
                        # Computes the weight.
                        aP = exp(-(pow(x-ai,2)+pow(y-aj,2))/18)
                        # Gets the HSV values from the point.
                        aC = self.hsv[aj][ai]
                        # Saves everything on the vectors.
                        P = np.append(P, aP)
                        hM = np.append(hM, aC[0] * aP)
                        sM = np.append(sM, aC[1] * aP)
                        vM = np.append(vM, aC[2] * aP)
                    except:
                        pass

            # Computes the normalizing factor.
            N = np.sum(P)
            # Computes the weighed sum of all HSV values.
            mH = np.sum(hM)
            mS = np.sum(sM)
            mV = np.sum(vM)

            # Refreshs the values for the Main Challenge Segmentation.
            if param == 'MAIN':
                self.main_upper[0] = int(max(min(mH/N + self.thrs, 255), self.main_upper[0]))
                self.main_lower[0] = int(min(max(mH/N - self.thrs, 0), self.main_lower[0]))
                self.main_upper[1] = int(max(min(mS/N + self.thrs, 255), self.main_upper[1]))
                self.main_lower[1] = int(min(max(mS/N - self.thrs, 0), self.main_lower[1]))
                self.main_upper[2] = int(max(min(mV/N + self.thrs, 255), self.main_upper[2]))
                self.main_lower[2] = int(min(max(mV/N - self.thrs, 0), self.main_lower[2]))

            # Refreshs the values for the Step Challenge Segmentation.
            if param == 'STEP':
                self.step_upper[0] = int(max(min(mH/N + self.thrs, 255), self.step_upper[0]))
                self.step_lower[0] = int(min(max(mH/N - self.thrs, 0), self.step_lower[0]))
                self.step_upper[1] = int(max(min(mS/N + self.thrs, 255), self.step_upper[1]))
                self.step_lower[1] = int(min(max(mS/N - self.thrs, 0), self.step_lower[1]))
                self.step_upper[2] = int(max(min(mV/N + self.thrs, 255), self.step_upper[2]))
                self.step_lower[2] = int(min(max(mV/N - self.thrs, 0), self.step_lower[2]))

            # Refreshs the values for the Swerve Challenge Segmentation.
            if param == 'SWERVE':
                self.swerve_upper[0] = int(max(min(mH/N + self.thrs, 255), self.swerve_upper[0]))
                self.swerve_lower[0] = int(min(max(mH/N - self.thrs, 0), self.swerve_lower[0]))
                self.swerve_upper[1] = int(max(min(mS/N + self.thrs, 255), self.swerve_upper[1]))
                self.swerve_lower[1] = int(min(max(mS/N - self.thrs, 0), self.swerve_lower[1]))
                self.swerve_upper[2] = int(max(min(mV/N + self.thrs, 255), self.swerve_upper[2]))
                self.swerve_lower[2] = int(min(max(mV/N - self.thrs, 0), self.swerve_lower[2]))

# --- Calibration of Main Challenge ---------------------------------------------------------------

    def MainCalibration(self):
        # Generates new window.
        cv2.namedWindow('Main Calibration')
        # Set the mouse event used.
        cv2.setMouseCallback('Main Calibration', self.Segment, param='MAIN')
        z = 0
        while True:
            if z == 0:
                # Takes a shot from camera.
                self.capture()
                z = 10
            else:
                z -= 1
            try:    
                # Generates a mask from the hsv image.
                mask = cv2.inRange(self.hsv, self.main_lower, self.main_upper)
                # Inverts mask.
                nmask = cv2.bitwise_not(mask)
                # Paints the mask into the image.
                res = cv2.bitwise_and(self.img, self.img, mask=nmask)
                try:
                    # Finds the contours on the mask.
                    _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    # Computes the areas of the contours.
                    areas = [cv2.contourArea(c) for c in contours]
                    # Finds the biggest area.
                    max_index = np.argmax(areas)
                    # Gets the contour with the biggest area.
                    cnt=contours[max_index]
                    # Computs the moments of the contour.
                    M = cv2.moments(cnt)
                    # Computes the centroid of the contour.
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # Draws a circle in the middle of the contour.
                    cv2.circle(res, (cx, cy), int(len(res)/30), [0,255,0], -1)
                except:
                    pass
                # Draws a small circle around the mouse.
                cv2.circle(res, self.pos, self.rad, [255,255,0], 2)
                # Shows the image on screen.
                cv2.imshow('Main Calibration', cv2.resize(res, (int(len(res[0]) * self.scl), int(len(res) * self.scl))))
            except:
                pass

            # Waits key pressing.
            k = cv2.waitKey(20) & 0xFF
            if k == 27: # ESC finishes.
                break
            elif k == ord('r'): # R resets the segmentation values.
                self.reset(0)

        # Closes everything.
        cv2.destroyAllWindows()

# --- Calibration of Step Challenge ---------------------------------------------------------------
    def StepCalibration(self):
        # Generates new window.
        cv2.namedWindow('Step Calibration')
        # Set the mouse event used.
        cv2.setMouseCallback('Step Calibration', self.Segment, param='STEP')
        z = 0
        while True:
            if z == 0:
                # Takes a shot from camera.
                self.capture()
                z = 10
            else:
                z -= 1
            try:
                # Generates a mask from the hsv image.
                mask = cv2.inRange(self.hsv, self.step_lower, self.step_upper)
                # Inverts mask.
                nmask = cv2.bitwise_not(mask)
                # Paints the mask into the image.
                res = cv2.bitwise_and(self.img, self.img, mask=nmask)
                try:
                    # Finds the contours on the mask.
                    _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    # Computes the areas of the contours.
                    areas = [cv2.contourArea(c) for c in contours]
                    # Finds the biggest area.
                    max_index = np.argmax(areas)
                    # Gets the contour with the biggest area.
                    cnt=contours[max_index]
                    # Computs the moments of the contour.
                    M = cv2.moments(cnt)
                    # Computes the centroid of the contour.
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # Draws a circle in the middle of the contour.
                    cv2.circle(res, (cx, cy), int(len(res)/30), [255,255,0], -1)
                except:
                    pass
                # Draws a small circle around the mouse.
                cv2.circle(res, self.pos, self.rad, [255,255,0], 2)
                # Shows the image on screen.
                cv2.imshow('Step Calibration', cv2.resize(res, (int(len(res[0]) * self.scl), int(len(res) * self.scl))))
            except:
                pass

            # Waits key pressing.
            k = cv2.waitKey(20) & 0xFF
            if k == 27: # ESC finishes.
                break
            elif k == ord('r'): # R resets the segmentation values.
                self.reset(1)

        # Closes everything.
        cv2.destroyAllWindows()

# --- Calibration of Swerve Challenge -------------------------------------------------------------

    def SwerveCalibration(self):
        # Generates new window.
        cv2.namedWindow('Swerve Calibration')
        # Set the mouse event used.
        cv2.setMouseCallback('Swerve Calibration', self.Segment, param='SWERVE')
        z = 0
        while True:
            if z == 0:
                # Takes a shot from camera.
                self.capture()
                z = 10
            else:
                z -= 1
            try:
                # Generates a mask from the hsv image.
                mask = cv2.inRange(self.hsv, self.swerve_lower, self.swerve_upper)
                # Inverts mask.
                nmask = cv2.bitwise_not(mask)
                # Paints the mask into the image.
                res = cv2.bitwise_and(self.img, self.img, mask=nmask)
                try:
                    # Divides the image into 7 scanlines.
                    dx = int(len(res[0])/7)
                    # Computes the x coordinates of the scanlines.
                    li = [int(dx/2), dx + int(dx/2), 2 * dx + int(dx/2), 3 * dx + int(dx/2), 4 * dx + int(dx/2), 5 * dx + int(dx/2), 6 * dx + int(dx/2)]
                    # Initializes the scanlines sizes.
                    lj = [0, 0, 0, 0, 0, 0, 0]

                    # Iterates through each scanline.
                    for i in range(len(li)):
                        up = 0 # Initial Up value.
                        down = len(res) - 1 # Initial Down value.
                        
                        # Executes until finding the initial mask point.
                        while True:
                            # Computes the mean of Up and Down.
                            p = int((down + up)/2)
                            
                            if mask[p][li[i]] == 255:
                                up = p # If it is a max value, refreshs Up.
                            else:
                                down = p # If not, refreshs Down.
                            
                            #If Up and Down are near enough, goes line by line.
                            if down - up < 30:
                                for j in range(up, down):
                                    if mask[j][li[i]] == 255 and mask[j+1][li[i]] == 0:
                                        lj[i] = j 
                                        break
                                break
                        # Draws the scan line.
                        cv2.line(res, (li[i], 1840), (li[i], lj[i]), [0, 255, 0], 2)

                    # Initialize a wheighed sum and a normalizing factor.
                    s = 0
                    n = 0
                    # For each scanline gets its sum.
                    for i in range(len(li)):
                        s += li[i] * lj[i]
                        n += lj[i]
                    # The point to swerve is given by this.
                    try:
                        cv2.circle(res, (int(s/n), int(len(res)/2)), int(len(res)/30), [255,0,255], -1)
                    except:
                        pass
                except:
                    pass
                # Draws a small circle around the mouse.
                cv2.circle(res, self.pos, self.rad, [255,255,0], 2)
                # Shows the image on screen.
                cv2.imshow('Swerve Calibration', cv2.resize(res, (int(len(res[0]) * self.scl), int(len(res) * self.scl))))
            except:
                pass

            # Waits key pressing.
            k = cv2.waitKey(20) & 0xFF
            if k == 27: # ESC finishes.
                break
            elif k == ord('r'): # R resets the segmentation values.
                self.reset(2)

        # Closes everything.
        cv2.destroyAllWindows()

# --- Running segmentation of the Main Challenge --------------------------------------------------

    def MainRunning(self, res):
        try:
            # Generates a mask from the hsv image.
            mask = cv2.inRange(self.hsv, self.main_lower, self.main_upper)
            # Finds the contours on the mask.
            _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # Computes the areas of the contours.
            areas = [cv2.contourArea(c) for c in contours]
            # Finds the biggest area.
            max_index = np.argmax(areas)
            # Gets the contour with the biggest area.
            cnt=contours[max_index]
            # Computs the moments of the contour.
            M = cv2.moments(cnt)
            # Computes the centroid of the contour.
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])            

            try:
                # Draws a circle in the middle of the contour.
                cv2.circle(res, (cx, cy), int(len(res)/30), [0,255,0], -1)
            except:
                pass

            # Return a value
            return cx, cy
        except:
            return -1, -1

# --- Running segmentation of the Step Challenge --------------------------------------------------

    def StepRunning(self, res):
        try:
            # Generates a mask from the hsv image.
            mask = cv2.inRange(self.hsv, self.step_lower, self.step_upper)
            # Finds the contours on the mask.
            _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # Computes the areas of the contours.
            areas = [cv2.contourArea(c) for c in contours]
            # Finds the biggest area.
            max_index = np.argmax(areas)
            # Gets the contour with the biggest area.
            cnt=contours[max_index]
            # Computs the moments of the contour.
            M = cv2.moments(cnt)
            # Computes the centroid of the contour.
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            try:
                # Draws a circle in the middle of the contour.
                cv2.circle(res, (cx, cy), int(len(res)/30), [255,255,0], -1)
            except:
                pass

            # Return a value
            return cx, cy
        except:
            return -1, -1

# --- Running segmentation of the Swerve Challenge ------------------------------------------------

    def SwerveRunning(self, res):
        try:
            # Generates a mask from the hsv image.
            mask = cv2.inRange(self.hsv, self.swerve_lower, self.swerve_upper)
            
            # Divides the image into 7 scanlines.
            dx = int(len(self.img[0])/7)
            # Computes the x coordinates of the scanlines.
            li = [int(dx/2), dx + int(dx/2), 2 * dx + int(dx/2), 3 * dx + int(dx/2), 4 * dx + int(dx/2), 5 * dx + int(dx/2), 6 * dx + int(dx/2)]
            # Initializes the scanlines sizes.
            lj = [0, 0, 0, 0, 0, 0, 0]
            
            # Iterates through each scanline.
            for i in range(len(li)):
                up = 0 # Initial Up value.
                down = len(self.img) - 1 # Initial Down value.
                
                # Executes until finding the initial mask point.
                while True:
                    # Computes the mean of Up and Down.
                    p = int((down + up)/2)
                    
                    if mask[p][li[i]] == 255:
                        up = p # If it is a max value, refreshs Up.
                    else:
                        down = p # If not, refreshs Down.
                    
                    #If Up and Down are near enough, goes line by line.
                    if down - up < 30:
                        for j in range(up, down):
                            if mask[j][li[i]] == 255 and mask[j+1][li[i]] == 0:
                                lj[i] = j 
                                break
                        break

            # Initialize a wheighed sum and a normalizing factor.
            s = 0
            n = 0
            # For each scanline gets its sum.
            for i in range(len(li)):
                s += li[i] * lj[i]
                n += lj[i]
            # The point to swerve is given by this.
            try:
                cv2.circle(res, (int(s/n), int(len(res)/2)), int(len(res)/30), [255,0,255], -1)
            except:
                pass

            # Return something
            return (s/n)
        except:
            return -1

# --- Main function -------------------------------------------------------------------------------
    
    # x stands for what will be executed.
    #    x = 0 - Run only main segmentation function.
    #    x = 1 - Run segmentation for the step challenge.
    #    x = 2 - Run segmentation for the swerve challenge.
    
    # image Shows the camera image on screen.
    
    def Run(self, x=0, image=False):
        # Calibration.
        self.MainCalibration()
        if x == 1:
            self.StepCalibration()
        elif x == 2:
            self.SwerveCalibration()

        # Main execution.

        if image:
            # Generates new window.
            cv2.namedWindow('Running')

        while True:
            # Captures a camera frame.
            self.capture()
            # Makes a image copy.
            if image:
                res = cv2.repeat(self.img, 1, 1)
            else:
                res = 0

            # Runs the main segmentation.
            M = self.MainRunning(res)
            if x == 1:
                # Runs the segmentation for the Step Challenge
                St = self.StepRunning(res)
            elif x == 2:
                # Runs the segmentation for the Swerve Challenge
                Sw = self.SwerveRunning(res)

            if image:
                # Shows image on screen.
                cv2.imshow('Running', cv2.resize(res, (int(len(res[0]) * self.scl), int(len(res) * self.scl))))

            print "Main:", M,
            if x == 1:
                print "- Step:", St
            elif x == 2:
                print "- Swerve:", Sw
            else:
                print

            # Wait 'q' to be pressed
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

        # Closes all windows.
        cv2.destroyAllWindows()
