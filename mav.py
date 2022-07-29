from pymavlink import mavutil
from time import sleep
import cv2
from pyzbar.pyzbar import decode
from PIL import Image

cap = cv2.VideoCapture(-1)

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
    

master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
print(master)

master.wait_heartbeat()

vehicle_mode('POSHOLD')

arm_vehicle()

while(True):
    
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("no", gray)
    
    if decode(frame) != []:
        print(decode(frame)[0].rect[1])
        vehicle_mode('LAND')
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


sleep(10)
vehicle_mode('LAND')



