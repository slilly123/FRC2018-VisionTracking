import libjevois as jevois
import cv2
import numpy as np

#May work best if use GRIP with camera at image in given lighting conditions to get ranges
#works at home
lowerBound=np.array([32,0,165])
upperBound=np.array([124,255,255])
#tighten the color range - works at school
#lowerBound=np.array([71,48,227])
#upperBound=np.array([98,255,255])

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
interpolation = cv2.INTER_AREA

# Calibration distance
# Focal Length = (measured pixels * known distance)/known width of tape
# Focal Length = (60 * 24 in)/2 in = 720
focalLength= 720
#Distance calculation
# distance (inches) = (focal length * known width) / pixel width

# Calibration angle
# resolution (640x480), camera FOV (65 deg), use linear interpolation
# FOV/sqrt(640^2 + 480^2)
# Each pixel represents 65/800 = .08125 deg
angleConversion=0.08125
# Distance from center (pixels) * angleConversion

class AutonTapeDetector:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("cdp", 100, jevois.LOG_INFO)
		        
        self.frame = 0 # a simple frame counter used to demonstrate sendSerial()

    # ###################################################################################################
    ## Process function with USB output using OBS
    def process(self, inframe, outframe):
        # Get the next camera image and convert it to OpenCV BGR:
        inimg = inframe.getCvBGR()

        # Start measuring image processing time
        self.timer.start()
       
        # resize
        inimg = cv2.resize(inimg,(640,480), 0, 0, interpolation)

        # convert BGR to HSV
        inimgHSV = cv2.cvtColor(inimg,cv2.COLOR_BGR2HSV)

        # create the Mask
        mask = cv2.inRange(inimgHSV,lowerBound,upperBound)
        
        # morphology
        maskOpen = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        
        # find contours
        im2,conts,h = cv2.findContours(maskClose,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        # loop over contours
        for i in range(len(conts)):
            perimeter = cv2.arcLength(conts[i],True)
            approx = cv2.approxPolyDP(conts[i],0.02*perimeter,True)

            # check contour verticies for rectangle, ideally there will be 4 but may be more at a distance
            if len(approx) <= 6:
               x,y,w,h=cv2.boundingRect(conts[i])
               # compare aspect ratio
               if ((w/h) < .25 and (w/h) > .1):
                  cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,0,255), 2)
                  targetCenterX = x + (w / 2)
                  targetCenterY = y + (h / 2)
                  distanceToTarget = (focalLength * 2)/w
                  jevois.sendSerial("Target # {} w {} distance (inches) {}".format(i, w, distanceToTarget))
                   
        # Write frames/s info from our timer into the edge map
        fps = self.timer.stop()
        
        outframe.sendCvBGR(inimg)

    # ###################################################################################################
    ## Process function with no USB output
    ## ******* Need to manually streamon, streamoff through Termite *******
    ## In initscript file:
    ## listmappings
    ## setmapping2 YUYV 640 480 15.0 JeVois AutonTapeDetector
    ## streamon
    ## streamoff

    def processNoUSB(self, inframe):

        # Get the next camera image and convert it to OpenCV BGR:
        inimg = inframe.getCvBGR()
       
        # Start measuring image processing time
        self.timer.start()

        # resize
        inimg = cv2.resize(inimg,(640,480), 0, 0, interpolation)

        # convert BGR to HSV
        inimgHSV = cv2.cvtColor(inimg,cv2.COLOR_BGR2HSV)

        # create the Mask
        mask = cv2.inRange(inimgHSV,lowerBound,upperBound)
        
        # morphology
        maskOpen = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        
        # find contours
        im2,conts,h = cv2.findContours(maskClose,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        # loop over contours
        for i in range(len(conts)):
            perimeter = cv2.arcLength(conts[i],True)
            approx = cv2.approxPolyDP(conts[i],0.02*perimeter,True)

            # check contour verticies for rectangle, ideally there will be 4 but may be more at a distance
            if len(approx) <= 6:
               x,y,w,h=cv2.boundingRect(conts[i])
               # compare aspect ratio
               if ((w/h) < .25 and (w/h) > .1):
                  cv2.rectangle(inimg,(x,y),(x+w,y+h),(0,0,255), 2)
                  targetCenterX = x + (w / 2)
                  targetCenterY = y + (h / 2)
                  distanceToTarget = (focalLength * 2)/w
                  jevois.sendSerial("Target # {} w {} distance (inches) {}".format(i, w, distanceToTarget))
                   
        # Write frames/s info from our timer into the edge map
        fps = self.timer.stop()

