from pymavlink import mavutil
from time import sleep
import cv2
import numpy
from pyzbar.pyzbar import decode
from PIL import Image

cap = cv2.VideoCapture(0)
if cap.isOpened() == 0:
    exit(-1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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


"""sleep(10)
vehicle_mode('LAND')"""
