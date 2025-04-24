#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import sys
import signal
import threading
import os

import argparse

parser = argparse.ArgumentParser(description = 'Control Copter and send commands in guided mode')
parser.add_argument('--connect',
                    help = 'Vehicle connection target string. If not specified, SITL will automatically start and be used') 
args = parser.parse_args()

#changed for mission planner connection
connection_string = 'tcp:127.0.0.1:56700' 
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready = True)

"""
Arms vehicle and takes off to specified altitude
"""
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Ensures user does not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Arming motors")
    # Copter should arm in guided mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    print("Vehicle is taking off!")
    # Take off to specified altitude
    vehicle.simple_takeoff(aTargetAltitude)
    

    #requires GPS
    """
    Wait until vehicle has reached specified altitude before processing next command
    Any command directly after Vehicle.simple_takeoff will execute immediately 
    """
    """
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached  target altitude")
            break
        time.sleep(1)
    """
    
#kill switch implementation
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
import threading

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



# Arm and take off to altitude of 1 meters
arm_and_takeoff(1)

#allow for time to reach a hover
time.sleep(20)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

print("Close vehicle object")
vehicle.close()

if sitl is not None:
    sitl.stop()

print("Mission Complete")

