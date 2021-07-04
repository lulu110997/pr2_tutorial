#!/usr/bin/env python

import cv2
import numpy as np

colour = np.uint8([[[0, 255, 255]]])
hsv_colour = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
print(hsv_colour)

lower_yellow = np.array([20, 2, 2])
upper_yellow = np.array([40, 255, 255])
# lower_orange = np.array([8, 100, 100])
# upper_orange = np.array([19, 255, 255])


cap = cv2.VideoCapture(0)

while True:
    __, frame = cap.read()  # Capture the image from camera

    # Apply a blur to reduce noise and then convert the BGR
    # BGR color space of captured image to HSV color space
    frame = cv2.GaussianBlur(frame,(7,7),1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Obtain a mask to 'zero out' the colour that is not of
    # interest, leaving an image with only the desired colours
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Find the contour of the shape of interest
    img = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    __, thresh = cv2.threshold(mask, 127, 255, 0)
    __, contours, hierarchy = cv2.findContours(thresh, 1, 2)

    try:
        # draw in blue the contours that were founded
        cv2.drawContours(img, contours, -1, 255, 3)

        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)

        # draw the biggest contour (c) in green
        cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2)
    except:
        pass

    # Show the original image, mask and masked image
    cv2.imshow('original', frame)
    # cv2.imshow('mask',mask)
    cv2.imshow('masked', res)

    k = cv2.waitKey(5) & 0xFF  # Press esc to exit
    if k == 27:
        break

cv2.destroyAllWindows()