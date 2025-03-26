"""
This is the beginning of the program using everything
to run the autonomous flying, detection, and communication
on the UAV drone.
"""

"""
TO-DO LIST HERE!!!:

- IF THINGS ARE ADDED, DEFINE FUNCTIONS FOR THEM TO MAKE EASIER USE
- ADD GOOD COMMENTS PLEASE; VERY IMPORTANT
- ADD THE REST OF THE CAMERA DETECTION CODE
- ADD CODE TO CONNECT TO THE UAV (CUBE ORANGE)
- ADD KILLSWITCH CODE TO MAKE SURE LANDING IS SAFE
- ADD CODE TO ARM/DISARM
- ADD CODE TO SEND GPS COORDINATES OF THE UAV
- ADD IMPORTS (AS NEEDED)
- ADD FLYING CAPABILITIES (IF POSSIBLE RN)

"""

"""    BEGINNING OF IMPORTS    """

# GENERAL IMPORTS
import time
import math
import sys
import signal
import threading
import os

# DRONE USE IMPORTS
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

# CAMERA DETECTION IMPORTS
import cv2
from picamera2 import Picamera2

"""    END OF IMPORTS   """

"""    BEGINNING OF CAMERA DETECTION    """

# Function to initialize the camera and its configurations
def camera_init():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    # Using smallest dictionary since there are only markers from ID 1 to ID 5
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParameters = cv2.aruco.DetectorParameters_create()

    # The dropzone variable can be changed to any ID; our current tests use marker 3 as the designated dropzone
    dropzone = 3