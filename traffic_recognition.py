# IMIMOD 2021 Life Hacks #
# Project: Scalable Robots #
# Team member:  Murugayah, Kanapathy, Balasubramaniam, Hari Chand, Abdullah, Azizul #

import cv2
import numpy as np
import time
from imutils.perspective import four_point_transform
import imutils

#camera = cv2.VideoWriter(0)
#camera = cv2.VideoCapture(0)
camera = cv2.VideoCapture('rtsp://192.168.0.125:8554/mjpeg/1')

def findTrafficSign():
    '''
    This function find blobs with blue color on the image. After blobs were found it detects the largest square blob, that must be the sign.
    '''
    # define range HSV for blue color of the traffic sign
    lower_blue = np.array([85,100,70])
    upper_blue = np.array([115,255,255])

    while True:
        (grabbed, frame) = camera.read() #grab the current frame from camera
        if not grabbed:
            print("No input image")
            break

        frame = imutils.resize(frame, width=500)
        frameArea = frame.shape[0]*frame.shape[1]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert color image to HSV color scheme
        kernel = np.ones((3,3),np.uint8) # define kernel for smoothing
        mask = cv2.inRange(hsv, lower_blue, upper_blue) # extract binary image with active blue regions
        # Perform morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # find contours in the mask

        # defite string variable to hold detected sign description
        detectedTrafficSign = None

        # define variables to hold values during loop
        largestArea = 0
        largestRect = None

        if len(cnts) > 0:
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # count euclidian distance for each side of the rectangle
                side_one = np.linalg.norm(box[0]-box[1])
                side_two = np.linalg.norm(box[0]-box[3])
                # count area of the rectangle
                area = side_one*side_two
                if area > largestArea:
                    largestArea = area
                    largestRect = box

        # draw contour of the found rectangle on  the original image
        if largestArea > frameArea*0.02:
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            warped = four_point_transform(mask, [largestRect][0]) # cut and warp interesting area
            detectedTrafficSign = identifyTrafficSign(warped) # use function to detect the sign on the found rectangle

            # write the description of the sign on the original image
            cv2.putText(frame, detectedTrafficSign, tuple(largestRect[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            print(detectedTrafficSign) 
            #Print out the results
            #Turn Left
            #Turn Right
            #Move Straight
            #Turn Back 

        # show original image
        cv2.imshow("Video Input", frame)

        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Program Stopped and Close All Windows")
            break

def identifyTrafficSign(image):
    '''
    In this function we select some ROI in which we expect to have the sign parts. If the ROI has more active pixels than threshold we mark it as 1, else 0
    After path through all four regions, we compare the tuple of ones and zeros with keys in dictionary SIGNS_LOOKUP
    '''

    # define the dictionary of signs segments so we can identify
    # each signs on the image
    SIGNS_LOOKUP = {
        (1, 0, 0, 1): 'Turn Right', # turnRight
        (0, 0, 1, 1): 'Turn Left', # turnLeft
        (0, 1, 0, 1): 'Move Straight', # moveStraight
        (1, 0, 1, 1): 'Turn Back', # turnBack
    }

    THRESHOLD = 150

    image = cv2.bitwise_not(image)
    (height, width) = np.divide(image.shape, 10)
    height = int(height)
    width = int(width)

    # mark the ROIs borders on the image
    cv2.rectangle(image, (width, 4*height), (3*width, 9*height), (0,255,0),2) # left block
    cv2.rectangle(image, (4*width, 4*height), (6*width, 9*height), (0,255,0),2) # center block
    cv2.rectangle(image, (7*width, 4*height), (9*width, 9*height), (0,255,0),2) # right block
    cv2.rectangle(image, (3*width, 2*height), (7*width, 4*height), (0,255,0),2) # top block

    # substract 4 ROI of the sign thresh image
    leftBlock = image[4*height:9*height, width:3*width]
    centerBlock = image[4*height:9*height, 4*width:6*width]
    rightBlock = image[4*height:9*height, 7*width:9*width]
    topBlock = image[2*height:4*height, 3*width:7*width]

    # we now track the fraction of each ROI
    leftFraction = np.sum(leftBlock)/(leftBlock.shape[0]*leftBlock.shape[1])
    centerFraction = np.sum(centerBlock)/(centerBlock.shape[0]*centerBlock.shape[1])
    rightFraction = np.sum(rightBlock)/(rightBlock.shape[0]*rightBlock.shape[1])
    topFraction = np.sum(topBlock)/(topBlock.shape[0]*topBlock.shape[1])

    segments = (leftFraction, centerFraction, rightFraction, topFraction)
    segments = tuple(1 if segment > THRESHOLD else 0 for segment in segments)

    #cv2.imshow("Warped", image)

    if segments in SIGNS_LOOKUP:
        return SIGNS_LOOKUP[segments]
    else:
        return None

def main():
    findTrafficSign()

if __name__ == '__main__':
    main()
