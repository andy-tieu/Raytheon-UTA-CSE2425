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

# Connect to the drone using the same TCP port as mission planner

connection_string = 'tcp:127.0.0.1:56700'  
print('Connecting to vehicle on: %s' % connection_string)

vehicle = connect(connection_string, wait_ready = True)


while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
vehicle.wait_ready()

# Arm the drone (example)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)
print("Vehicle armed!")

time.sleep(5)

print("disarming...")
vehicle.armed = False

while vehicle.armed:
        print("Waiting for vehicle to disarm...")
        time.sleep(1)
print("vehicle disarmed!")

vehicle.close()

print("test complete")


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
