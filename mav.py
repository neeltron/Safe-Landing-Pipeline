# -*- coding: utf-8 -*-
"""
Created on Tue Aug  2 15:11:06 2022

@author: Neel
"""
from pymavlink import mavutil
from time import sleep
import cv2
import numpy
from pyzbar.pyzbar import decode
from PIL import Image

webcam = cv2.VideoCapture(0)

webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def arm_vehicle():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
    

def vehicle_mode(input_mode):
    mode = input_mode
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)


    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)
print(master)

master.wait_heartbeat()

vehicle_mode('STABILIZE')

arm_vehicle()

while True:
    
    _, imageFrame = webcam.read()
    left_right_image = numpy.split(imageFrame, 2, axis=1)
    
    hsvFrame = cv2.cvtColor(left_right_image[1], cv2.COLOR_BGR2HSV)
  
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
  
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
  
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
      
    kernal = np.ones((5, 5), "uint8")
      
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
      
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
      
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask = blue_mask)
   
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)
              
            cv2.putText(imageFrame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))    
  
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    
    cv2.imshow("Multiple Color Detection in Real-TIme", left_right_image[1])
    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break
    
    retval, frame = cap.read()
    left_right_image = numpy.split(frame, 2, axis=1)
    """cv2.imshow("frame", frame)
    cv2.imshow("right", left_right_image[0])"""
    cv2.imshow("left", left_right_image[1])
    
    if decode(left_right_image[1]) != []:
        print(decode(left_right_image[1])[0].rect[1])
        vehicle_mode('LAND')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  
