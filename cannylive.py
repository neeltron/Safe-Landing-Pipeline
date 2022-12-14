# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 13:48:47 2022

@author: Neel
"""
import cv2
 
import numpy as np
 
vid = cv2.VideoCapture(0)

vid.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
 
while(1):
    ret, frame = vid.read()
    cap = np.split(frame, 2, axis=1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([30,150,50])
    upper_red = np.array([255,255,180])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow('Original',frame)
    edges = cv2.Canny(frame,100,200)
    cv2.imshow('Edges',edges)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
 
 
cap.release()
 
cv2.destroyAllWindows()
