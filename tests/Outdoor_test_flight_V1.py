#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import signal
import threading
import os

"""
# This Script does the following:
# 1.) Connects to the Cube Orange via serial on /dev/ttyAMA0.
# 2.) Waits for the drone to be ready.
# 3.) Arms the motors.
# 4.) Takes off to 2 meter altitude.
# 5.) Waits until the altitude is reached.
# 6.) Drone is given instruction to go straight 2 meters at a speed of 0.25m/s
# 7.) Lands the drone.
# 8.) Closes the connection and exits.
NOTE: This script NEEDS GPS to function. UAV will not switch to guided mode without GPS Lock.
KILL SWITCH: 
    kill switch is either 'ctrl + c' or 'k' and pressing enter
"""


#COMMENTED OUT CODE IS FOR MISSION PLANNER SIMULATION TESTING (ASK SIMON)
"""
connection_string = 'tcp:127.0.0.1:57600'  
print('Connecting to vehicle on: %s' % connection_string)

vehicle = connect(connection_string, wait_ready = True)
"""
# Connect to the drone
print("Connecting to UAV CubeOrange via /dev/ttyAMA0...")
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
print("UAV RPi is successfully connected to the Cube Orange!\n")



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
    time.sleep(4)
    


"""
Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
specified `original_location`. The returned LocationGlobal has the same `alt` value
as `original_location`.

The function is useful when you want to move the vehicle around specifying locations relative to 
the current vehicle position.

The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
"""

def get_location_metres(original_location, dNorth, dEast):
    # 'Spherical' radius of earth
    earth_radius = 6378137.0
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius * math.cos(math.pi * original_location.lat/180))

    # New position in decimal degrees
    newLat = original_location.lat + (dLat * 180/math.pi)
    newLon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newLat, newLon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newLat, newLon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation

"""
Returns the ground distance in metres between two LocationGlobal objects.

This method is an approximation, and will not be accurate over large distances and close to the 
earth's poles.
"""
def get_distance_metres(aLocation1, aLocation2):
    dLat = aLocation2.lat - aLocation1.lat
    dLon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dLat * dLat) + (dLon * dLon)) * 1.113195e5

"""
Returns the bearing between the two LocationGlobal objects passed as parameters.
This method is an approximation, and may not be accurate over long distances and
close earths poles.
"""
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

"""
Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
"""
def goto_position_target_global_int(aLocation):

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, 0, 0, # X,Y,Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    print(msg)
    vehicle.send_mavlink(msg)

"""
Moves the vehicle to a postiion dNorth meters North and dEast meters East
of the current position.
The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
the target position. This allows it to be called with different position-setting commands.
This method reports the distance to target every two seconds
"""
def goto(dNorth, dEast, Gspeed, gotoFunction = vehicle.simple_goto):
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    vehicle.groundspeed = Gspeed
    #Stop action if we are no longer in guided mode
    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance*0.05:
            print("Target reached")
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
#----------------------------------------------------------------------------

# Arm and take off to altitude of 2 meters
arm_and_takeoff(2.01)

print("Fly straight line path to 2 meters")
print("Setting groundspeed to 0.2 m/s")
#modified for less distance
Gspeed = 0.25 #set both Gspeed and vehicle.groundspeed to same value
vehicle.groundspeed = 0.25
#set to 27.41 for 30 meters
goto(2,0, Gspeed, goto_position_target_global_int)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(2)

print("Close vehicle object")
vehicle.close()
print("Mission Complete")

