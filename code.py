from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

speed = 0.1 # [m/s]

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the vehicle
print("Connecting to vehicle on: %s" % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

def arm_and_takeoff(altitude):
    
    #while not vehicle.is_armable:
        #print("waiting to be armable")
        #time.sleep(1)
    
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True
    
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed")
    
    print("Taking off")
    #vehicle.simple_takeoff(altitude)
    
    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.lf m" %v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)

def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9: az
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

# Function to compute drone action
def compute_drone_action(x1, y1, x2, y2):
    
    turning = ""
    moving = ""
    elvevate = ""
    
    width = 640.0
    height = 480.0
    
    area, centre = (x2-x1)*(y2-y1), [(x1+x2)/2, (y1+y2)/2]
    
    area = abs(area)
    new_area = area*100/(width*height)
    print(new_area)
    
    normalized_centre = [None, None]
    
    normalized_centre[0] = centre[0] / width
    normalized_centre[1] = centre[1] / height
    
    if normalized_centre[0] > 0.5:
        print("turn tight")
        set_velocity_body(vehicle, 0, speed, 0)
    elif normalized_centre[1] < 0.2:
        print("turn left")
        set_velocity_body(vehicle, 0, -speed, 0)
        
    #if normalized_centre[1] > 0.4:
        #print("go down")
        #set_vehicle_body(vehicle, 0, 0, -speed)
    #elif normalized_centre[1] < 0.2:
        #print("go up")
        #set_vehicle_body(vehicle, 0, 0, speed)
        
    if new_area > 15:
        print("go backwards")
        set_velocity_body(vehicle, -speed, 0, 0)
    elif new_area < 1:
        print("go ahead")
        set_velocity_body(vehicle, speed, 0, 0)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.vflip = True
camera.hflip = True
    
rawCapture = PiRGBArray(camera, size = (640, 480))

time.sleep(0.1)

arm_and_takeoff(1.5)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    img = frame.array

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([160, 100, 100])
    upper_red = np.array([179, 255, 255])

    red = cv2.inRange(hsv, lower_red, upper_red)
    kernal = np.ones((5, 5), "uint8")
    blue = cv2.dilate(red, kernal)

    res = cv2.bitwise_and(img, img, mask = red)
    
    contours, heirarchy = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    max_brightness = 0
    for cnt in contours:
        rect = cv2.boundingRect(cnt)
        x, y, w, h = rect
        if w*h > 20000:
            mask = np.zeros(img.shape, np.uint8)
            mask[y:y+h, x:x+w] = img[y:y+h, x:x+w]
            brightness = np.sum(mask)
            if brightness > max_brightness:
                brightest_rectangle = rect
                max_brightness = brightness
    
                x, y, w, h = brightest_rectangle
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 5)
                x1, y1, x2, y2 = x, y, w, h
                compute_drone_action(x1, y1, x2, y2)
    
    cv2.imshow("Colour Tracking", img)
    img = cv2.flip(img, 1)
    cv2.imshow("Red", res)
    
    key = cv2.waitKey(1)
    
    rawCapture.truncate(0)
    
    if key == ord("q"):
        break
