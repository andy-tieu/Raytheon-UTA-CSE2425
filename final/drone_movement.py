"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO RUN UAV DRONE MOVEMENT
NOTE THAT INCLUDES A TENATIVE IMPLEMENTATION FOR MOVEMENT INDOORS WHILE SLAM IS FULLY DEVELOPED AND TESTED
THIS BODY VELOCITY METHOD IS NOT THE ONE CALLED IN CAMERA DETECTION PROGRAM, WHICH HAS ITS OWN METHOD FOR ENTERING GUIDE MODE WHEN CORRECT MARKER IS FOUND
"""

""" START OF IMPORTS """

# Dependencies; need to be installed
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

import time
# for command line parameter parsing
import signal
import os
import argparse
import sys
import threading
import camera_detection

""" END OF IMPORTS """



""" START OF GLOBALS """

debug = True                    # Debug mode: For verifying when logging occurs, turn off when not debugging

""" START OF GLOBALS """



""" START OF FUNCTIONS """


# Function to perform connection between RPi and Cube Orange on the UAV
def drone_connect(conn_str):
    print("Connecting to UAV CubeOrange via /dev/ttyAMA0...")
    vehicle = connect(conn_str, baud=57600, wait_ready=True)
    print("UAV RPi successfully connected to the Cube Orange!\n")

    return vehicle

def arm():
    print("Basic pre-arm checks")
    # Ensures user does not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Arming motors")
    # Copter should arm in poshold mode, note that it should not take off yet
    vehicle.mode = VehicleMode("POSHOLD")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    # Now switch to AUTO mode to start the mission
    vehicle.mode = VehicleMode("AUTO")
    while not vehicle.mode.name == "AUTO":
        print(f"Waiting for mode change... Current mode: {vehicle.mode.name}")
        time.sleep(1)
        
    print("Vehicle is armed, ready for takeoff in AUTO mode")
    vehicle_logging.log_start()  # Start logging mission


# Function to arm and rise to specified altitude
def arm_and_takeoff(vehicle, target_alt):
    print("Basic pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    print("Vehicle is taking off!")
    # Take off to specified altitude
    vehicle.simple_takeoff(target_alt)

    """
    Wait until vehicle has reached specified altitude before processing next command
    Any command directly after Vehicle.simple_takeoff will execute immediately 
    """
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_alt * 0.95:
            print("Reached  target altitude")
            break
        time.sleep(1)
    time.sleep(4)

def send_ned_velocity( vehicle, velocityX, velocityY, velocityZ):
	"""
	Move vehicle in direction specified by velocity vectors
	Helper method 
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,      # time_boot_ms (not used)
		0, 0,   # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (ensures speed and not position is enabled)
		0, 0, 0,    # position x, y, z (not used)
		velocityX, velocityY, velocityZ, # velocity for x, y, z in m/s
		0, 0, 0,    # accelleration (not used/not supported)
		0, 0)       # yaw (also not supported)
	# send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

# working method for our current search pattern, in consideration for the boundary search pattern of running the script with command line parameters
# to move physically in a snake search pattern, performing the next subsequent action outlined by the paramter after hitting the boundary
# note that this method uses body frame velocities to move the drone in a specific direction based on the forward, right, and down speeds
# this makes the movement relative to the drone instead of relying on compass directions from the GPS

def send_body_velocity(vehicle, forward_speed, right_speed, down_speed, check_interval=0.01):
    """
    Continuously sends body-frame velocity commands (m/s) until 'check_marker_func' returns True.
    - forward_speed, right_speed, down_speed: velocity along the drone's body axes
    - check_marker_func: a function that returns True if the marker/boundary is found, otherwise False
    - check_interval: how often to check the marker (seconds)

    This function does NOT stop until check_marker_func() is True.
    Once True, it sends a stop (zero velocity) command and returns.
    """

    if debug:
        print(f"Starting indefinite body-frame velocity: forward={forward_speed}, right={right_speed}, down={down_speed}")

    # Set up a MAVLink message for controlling velocities in BODY_NED frame
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,        # time_boot_ms (ignored)
        0, 0,     # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame = body (drone-centric) + NED
        0b0000111111000111,  # bitmask: use only the velocity components
        0, 0, 0,             # x, y, z positions (ignored)
        forward_speed,       # velocity in X (forward)
        right_speed,         # velocity in Y (right)
        down_speed,          # velocity in Z (down)
        0, 0, 0,             # accelerations (ignored)
        0, 0                 # yaw, yaw_rate (ignored)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

    if debug:
        print("Starting indefinite body-frame velocity movement...")
        print(f"Forward speed: {forward_speed} m/s, Right speed: {right_speed} m/s, Down speed: {down_speed} m/s\n")
    
    # Keep sending the command at ~10 Hz for the given duration
    # while True:
    #     # Send velocity command
    #     vehicle.send_mavlink(msg)
    #     vehicle.flush()

    #     # Check if marker is found
    #     if check_marker_func():
    #         if debug:
    #             print("Marker detected! Stopping movement...")
    #         break

    #     time.sleep(check_interval)

    # # After the duration, send a zero-velocity command to stop
    # stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #     0, 0, 0,
    #     mavutil.mavlink.MAV_FRAME_BODY_NED,
    #     0b0000111111000111,
    #     0, 0, 0,
    #     0, 0, 0,  # zero velocity
    #     0, 0, 0,
    #     0, 0
    # )
    # vehicle.send_mavlink(stop_msg)
    # vehicle.flush()
    # if debug:
    #     print("Sent zero velocity command to stop the drone.")


#kill switch implementation --------------------------------------------
def emergency_land(signal, frame):
    """ Emergency landing function triggered by CTRL+C """
    print("\n Kill switch activated! Landing immediately...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.flush()  # Ensure MAVLink command is sent

    print("Stopping script...")
    while vehicle.mode.name != "LAND":
        print("Waiting for mode change...")
        time.sleep(1)
    
    print("Mode successfully switched to LAND!")
    vehicle.close()
    os._exit(0)

# # Attach the kill switch to SIGINT (CTRL+C)
# signal.signal(signal.SIGINT, emergency_land)

def listen_for_kill():
    """ Monitors for user input to trigger an emergency stop. """
    while True:
        try:
            if input.lower() == "k":
                print("\n Alternate Kill switch activated! Landing immediately...")
                emergency_land(None, None)
        except EOFError:
            # avoid crashing if input stream closes unexpectedly
            break


def argParser():
            os._exit(0)  # Forcefully exit the script

# Start the listener in a separate thread
kill_thread = threading.Thread(target=listen_for_kill, daemon=True)
kill_thread.start()

def land(self):
    self.vehicle.mode = VehicleMode("LAND")

    while self.vehicle.mode.name != "LAND":
        print("Waiting for mode change...")
        time.sleep(1)

    print("Mode successfully switched to LAND!")

    """
    Function to parse command-line parameters and start the drone search pattern.
    Command-line parameters represent the movement instructions for north, south, west, and east.
    """
    parser = argparse.ArgumentParser(description="Drone Movement Program for Drop Zone and Boundary Detection.")
    parser.add_argument("northinstructions", help="Instructions for north movement.")
    parser.add_argument("southinstructions", help="Instructions for south movement.")
    parser.add_argument("westinstructions", help="Instructions for west movement.")
    parser.add_argument("eastinstructions", help="Instructions for east movement.")
    args = parser.parse_args()

""" END OF FUNCTIONS"""
