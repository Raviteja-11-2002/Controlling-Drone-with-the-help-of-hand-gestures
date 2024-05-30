import cv2
import time
import os
import HandTrackingModule as htm
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
from pymavlink import mavutil
from time import sleep
import numpy as np

###################################################################################

def connectMyCopter():
    parser=argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args=parser.parse_args()

    connection_string=args.connect
    baud_rate=921600

    vehicle=connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

###################################################################################
# Function to arm and takeoff

def arm_and_takeoff(TargetAltitude):

    # Switch vehicle to Guided Mode
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!="GUIDED":
        print("Waiting for guided mode")
        time.sleep(1)

    # Arming the Vehicle
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for the vehicle to be armed")
        time.sleep(1)

    vehicle.simple_takeoff(TargetAltitude)

    while True:
        print("Current Altitude: %d" , vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude*.95:
            break
        time.sleep(1)

    print("Target Altitude reached")
    return None

##################################################################
#-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
   
    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:
   
   
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
###################################################################
vehicle = connectMyCopter()

wCam, hCam = 640, 480
deadZone = 100
pTime = 0
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)
detector = htm.handDetector(detectionCon=0.8, maxHands=1)
x = [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57]
y = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
coff = np.polyfit(x, y, 2)   # y = AX^2 + BX + C
c = []
i = 0
tipIds = [4, 8, 12, 16, 20]
while True:
    success, img = cap.read()
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)
    #print(lmList)

    if len(lmList) !=0:
        fingers = []
        # Thumb . Here the x value of thumb tip is compared with the x value of mid thumb
        if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
            fingers.append(1)

        else:
            fingers.append(0)

        # Other Fingers
        for id in range(1,5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id]-2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        #print(sum(fingers))
        
        x1, y1 = lmList[5][1], lmList[5][2]
        x2, y2 = lmList[17][1], lmList[17][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        ty = lmList[4][2]
        #print(cx, cy)
        cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
        #length = int(math.hypot(x2 - x1, y2 - y1))
        #A, B, C = coff
        #distanceCM = A*length**2 + B*length + C
        #print(distanceCM)

        if sum(fingers) == 0:
            print(" Arm and Takeoff ")
            arm_and_takeoff(2)
        
        if sum(fingers) == 5:
            
            if ((cx < int(wCam/2) + deadZone) and (cx > int(wCam/2) - deadZone)):
                print("Hold Position")
                set_velocity_body(vehicle, 0, 0, 0)
                
            if (cx < int(wCam/2) - deadZone):
                print("Moving Right")
                set_velocity_body(vehicle, 0, 0.5, 0)
                
                
            if (cx > int(wCam/2) + deadZone):
                print("Moving Left")
                set_velocity_body(vehicle, 0, -0.5, 0)
                
         
        if sum(fingers) == 1:
            
            if ((ty < int(hCam/2) + deadZone) and (ty > int(hCam/2) - deadZone)):
                print("Hold Position")
                set_velocity_body(vehicle, 0, 0, 0)
                
            if (ty < int(hCam/2) - deadZone):
                print("Moving Up")
                set_velocity_body(vehicle, 0, 0, -1)
                
            if (ty > int(hCam/2) + deadZone):
                print("Moving Down")
                set_velocity_body(vehicle, 0, 0, 1)
            
            
    
        #if sum(fingers) == 5:
         #   c.append(cx)
          #  if len(c)!=0:
           #     for i in range(len(c)):
            #        difference = c[i]-c[i-1]
                    #print(difference)
             #       if difference > 0:
              #          print("Moving Left")
               #         set_velocity_body(vehicle, 0, -3, 0)
                #    elif difference < 0:
                 #       print("Moving Right")
                  #      set_velocity_body(vehicle, 0, 3, 0)
                   # elif difference == 0:
                    #    print("Hold Position")
                     #   set_velocity_body(vehicle, 0, 0, 0)
            #
            #print(" Moving Right ")
            #set_velocity_body(vehicle, distanceCM*0.05, 0, 0)


        

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (40, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 3)
    cv2.imshow("Image", img)
    cv2.waitKey(1)


