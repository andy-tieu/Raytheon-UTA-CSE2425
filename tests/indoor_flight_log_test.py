from dronekit import connect, VehicleMode
import time
import signal
import os

# This script does the following:
# 1.) Connects to the Cube Orange via serial on /dev/serial0.
# 2.) Logs UAV telemetry data to a file every 2 seconds using a timer ISR.
# 2.) Waits for the drone to be ready, press ENTER to start the test.
# 3.) "Arms" the motors, throttle MUST be controlled manually or via RC.
# 4.) Takes off via manual input
# 5.) Waits until the altitude is reached.
# 6.) Hovers for 5 seconds.
# 8.) Commands an automatic landing
# 9.) Closes the connection and exits.

# NOTE: This script is designed for indoor testing
# To do so, set ARMING_CHECK = 0 and GPS_ALLOW_NO_FIX = 1 in Mission Planner, this program will not operate without this override

# Connect to the drone
print("Connecting to UAV Cube Orange via /dev/serial0...")
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
print("Connected to Cube Orange\n")

log_file = "indoor_testlog.txt"

# ISR logging function
def log_drone_status(signum, frame):
    with open(log_file, "a") as f:
        f.write(f"Time: {time.strftime('%H:%M:%S')}\n")
        f.write(f"Mode: {vehicle.mode.name}\n")
        f.write(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m\n")
        f.write(f"Velocity: {vehicle.velocity}\n")
        f.write(f"GPS: {vehicle.location.global_frame}\n")
        f.write("-" * 30 + "\n")

# Set up ISR to log every 2 seconds
signal.signal(signal.SIGALRM, log_drone_status)
signal.setitimer(signal.ITIMER_REAL, 2, 2)

# Emergency land function
def emergency_land(signal, frame):
    print("\nEmergency stop activated. Landing...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(2)
    vehicle.close()
    os._exit(0)

signal.signal(signal.SIGINT, emergency_land)

# Indoor test sequence (ALT_HOLD recommended)
print("Make sure the drone is in ALT_HOLD or STABILIZE mode for indoor testing.")
input("Press ENTER to arm the drone when ready (props installed, safety checks done).\n")

print("Arming motors...")
vehicle.armed = True

# Include segment if wanting to test GUIDED mode indoors without the use of GPS
# Note that this may fail at the command line if rejected
# IMPORTANT: ALso may lead to altitude instability or horiizontal drift, please use with caution
'''
print("Arming motors...")
vehicle.mode = VehicleMode("GUIDED")  # Attempting GUIDED; ensure GPS override parameters are set
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for drone to arm...")
    time.sleep(1)

print("Armed! Attempting to take off to 1 meter altitude.")
vehicle.simple_takeoff(1)  # Command automatic takeoff to 1 meter

# Monitor altitude until target reached
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {current_altitude:.2f} m")
    if current_altitude >= 0.95:
        print("Reached target altitude.")
        break
    time.sleep(1)
'''

while not vehicle.armed:
    print("Waiting for drone to arm...")
    time.sleep(1)

print("Drone armed. Waiting for altitude > 0.3 meters to start hover timer.")

# Wait until altitude crosses threshold
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude > 0.5:
        print(f"Altitude detected: {current_altitude:.2f} m. Starting 60-second hover timer.")
        break
    time.sleep(1)

# Start hover timer after drone is airborne
time.sleep(60)

print("Hover time complete. Initiating landing.")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.1:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m - Descending...")
    time.sleep(1)

print("Landed successfully. Closing connection.")
vehicle.close()
print("Indoor test complete.")

