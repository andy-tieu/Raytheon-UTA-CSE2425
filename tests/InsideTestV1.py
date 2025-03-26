from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import sys
import signal
import threading
import os

"""
# This Script does the following:
# 1.) Connects to the Cube Orange via serial on /dev/ttyAMA0.
# 2.) Waits for the drone to be ready.
# 3.) Arms the motors.
# 4.) Takes off to 1 meter altitude.
# 5.) Waits until the altitude is reached.
# 6.) Hovers for 10 seconds.
# 7.) Lands the drone.
# 8.) Closes the connection and exits.
NOTE: This script is attempting to use only the builtin altitude sensing with the cube orange and vehicle mode 'GUIDED_NOGPS' to function for indoor flight
      IF script does not work then 'vehicle.simple_takeoff()' is not working and altitude needs to be coded manually for take off and hover
KILL SWITCH: 
    kill switch is either 'ctrl + c' or 'k' and pressing enter
"""

# Connect to the drone
print("Connecting to UAV CubeOrange via /ddev/ttyAMA0...")
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
print("UAV Pi is successfully connected to the Cube Orange!\n")

"""
Arms vehicle and takes off to specified altitude
"""
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    #Should be able to work with this not commented out but for testing, this part being skippable is fine for now
    """
    # Ensures user does not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    """
    
    print("Arming motors")
    # Copter should arm in guided_nogps mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    time.sleep(2)
    print(vehicle.mode)
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    print("Vehicle is taking off!")
    # Take off to specified altitude
    vehicle.simple_takeoff(aTargetAltitude)
    

    """
    Wait until vehicle has reached specified altitude before processing next command
    Any command directly after Vehicle.simple_takeoff will execute immediately 
    """
    
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached  target altitude")
            break
        time.sleep(1)
    
    
#kill switch implementation --------------------------------------------
def emergency_land(signal, frame):
    """ Emergency landing function triggered by CTRL+C """
    print("\n Kill switch activated! Landing immediately...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.flush()  # Ensure MAVLink command is sent
    time.sleep(1)
    vehicle.close()
    os._exit(0)

# Attach the kill switch to SIGINT (CTRL+C)
signal.signal(signal.SIGINT, emergency_land)

def listen_for_kill():
    """ Monitors for user input to trigger an emergency stop. """
    while True:
        user_input = input()
        if user_input.lower() == "k":
            print("\n Kill switch activated! Landing immediately...")
            vehicle.mode = VehicleMode("LAND")
            vehicle.flush()  # Ensure MAVLink command is sent
            print("Stopping script...")
            time.sleep(2)  # Give some time for LAND mode to engage
            vehicle.close()  # Close connection safely
            
            print("Done")
            os._exit(0)  # Forcefully exit the script

# Start the listener in a separate thread
kill_thread = threading.Thread(target=listen_for_kill, daemon=True)
kill_thread.start()
#----------------------------------------------------------------------------

#MAIN

# Arm and take off to altitude of 1 meters
arm_and_takeoff(1)

#HOVER TIME including reaching target altitude
time.sleep(10)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(5)

print("Close vehicle connection")
vehicle.close()
print("Mission Complete")









