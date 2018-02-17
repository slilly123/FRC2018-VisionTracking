import libjevois as jevois
import cv2
import numpy as np

# from GRIP
#lowerBound=np.array([31,96,110])
#upperBound=np.array([53,255,255])
lowerBound=np.array([31,98,123])
upperBound=np.array([56,255,255])

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

# from GRIP
interpolation = cv2.INTER_CUBIC
min_area = 1000.0
min_perimeter = 100.0
min_width = 100.0
max_width = 500.0
min_height = 100.0
max_height = 500.0
solidity = [0.0, 100.0]
max_vertices = 1000000.0
min_vertices = 0.0
min_ratio = 0.8
max_ratio = 1.6

class CubeDetector:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("cdp", 100, jevois.LOG_INFO)
		
        self.frame = 0 # a simple frame counter used to demonstrate sendSerial()

    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        # Get the next camera image and convert it to OpenCV BGR
        inimg = inframe.getCvBGR()

        # Start measuring image processing time
        self.timer.start()
       
        # resize
        inimg = cv2.resize(inimg,(640,480), 0, 0, interpolation)

        # convert BGR to HSV
        inimgHSV = cv2.cvtColor(inimg,cv2.COLOR_BGR2HSV)

        # create the Mask
        mask = cv2.inRange(inimgHSV,lowerBound,upperBound)
        
        # blur (blur type from GRIP)
        radius = 15 #from GRIP (try other values)
        ksize = int(2 * round(radius) + 1)
        blurout = cv2.medianBlur(mask, ksize)
        
        # find contours
        im2,conts,h = cv2.findContours(blurout, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # filter contours
        output = []
        for contour in conts:
            x,y,w,h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertices or len(contour) > max_vertices):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)

        # out video is processed in video
        outimg = inimg

        #height, width
        maxArea = 0
        centerCubeX = 0
        centerCubeY = 0
        x = 0
        y = 0
        h = 0
        w = 0
        for contour in output:
            area = cv2.contourArea(contour)
            if (area > maxArea) :
                maxArea = area;
                x,y,w,h = cv2.boundingRect(contour)
                centerCubeX = x + (w / 2)
                centerCubeY = y + (h / 2)

        # draw rectangle around object
        cv2.rectangle(outimg,(x,y),(x+w,y+h),(0,0,255), 2)
        jevois.sendSerial("Contour, Target x,y {} {}".format(centerCubeX, centerCubeY))
                
        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()

        # send out video
        outframe.sendCvBGR(outimg)
