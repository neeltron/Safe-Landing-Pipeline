# -*- coding: utf-8 -*-
"""
Created on Mon Aug  8 08:33:00 2022

@author: Neel
"""
from pymavlink import mavutil
from time import sleep
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image


vid = cv2.VideoCapture(0)

vid.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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
    
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)
    

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=921600)
print(master)

master.wait_heartbeat()

vehicle_mode('POSHOLD')

arm_vehicle()


while(True):
    ret, frame = vid.read()
    left_right_image = np.split(frame, 2, axis=1)
    rawImage = cv2.imread(left_right_image[1]) 
    hsv = cv2.cvtColor(rawImage, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0,3,240])
    higher_white = np.array([255,5,255])

    white_range = cv2.inRange(hsv, lower_white, higher_white)
    
    tempx = 0

    for iter in white_range:
        count = 0
        for i in iter:
            count += 1
            if i != 0:
                print(i, count)
                tempx = i

    print(len(white_range))
    
    tempy = 0
    white_range2 = np.transpose(white_range)
    for iter in white_range2:
        count = 0
        for i in iter:
            count += 1
            if i != 0:
                print(i, count)
                tempy = i
                
    if tempx > 800:
        set_rc_channel_pwm(2, 1600)
        
    if tempx < 480:
        set_rc_channel_pwm(2, 1400)
        
    if tempy > 520:
        set_rc_channel_pwm(3, 1400)
        
    if tempy < 200:
        set_rc_channel_pwm(3, 1600)
    
    cv2.imshow('image', white_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


