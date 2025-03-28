"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO RUN UAV DRONE MOVEMENT
"""

""" START OF IMPORTS """

# Dependencies; need to be installed
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

# Basic imports
import time

""" END OF IMPORTS """



""" START OF FUNCTIONS """

# TODO:
# ADD CODE HERE TO PERFORM MOVEMENT COMMANDS

# Function to perform connection between RPi and Cube Orange on the UAV
def drone_connect(conn_str):
    print("Connecting to UAV CubeOrange via /dev/ttyAMA0...")
    vehicle = connect(conn_str, baud=57600, wait_ready=True)
    print("UAV RPi successfully connected to the Cube Orange!\n")

    return vehicle

# Function to arm and rise to specified altitude
def arm_and_takeoff(vehicle, target_alt):
    print("Basic pre-arm checks")

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")
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

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, vehicle):
    """ Move drone using velocity commands (m/s) for a specific duration """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # Target system, target component, coordinate frame
        0b0000111111000111,  # Bitmask: Only control velocities
        0, 0, 0,  # Position (ignored)
        velocity_x, velocity_y, velocity_z,  # Velocity (m/s)
        0, 0, 0,  # Acceleration (ignored)
        0, 0  # Yaw, yaw rate (ignored)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

""" END OF FUNCTIONS"""