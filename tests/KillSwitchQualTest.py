from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import signal
import os

# The intent of this script is perform basic, autonomous flight that tests the kill switch command ('CTRL + C' or 'k') to land the UAV.
# Successful use of the program will fulfill the Kill Switch Landing QualTest

# This script does the following:
# 1.) Connects to the Cube Orange via serial on /dev/serial0.
# 2.) Waits for the drone to be ready.
# 3.) Arms the motors.
# 4.) Takes off to about 3 meters initally to reach a safe floor.
# 5.) Takes off to 7 meters altitude (about 22 feet, enough to fulfill QualTest of 20 ft) at a speed of 2.3 m/s (about 5.1 mph, fulfill QualTest requirement of 5 mph)
# 6.) Waits until the altitude is reached.
# 7.) Hovers for 12 seconds.
# 8.) Logs UAV telemetry data to a file every 2 seconds using a timer ISR.
# 9.) Lands the drone within 5 yards of the takeoff point.
# 10.) Closes the connection and exits.
#
# Note: This script needs GPS to function. UAV will not switch to guided mode without GPS Lock.
# To kill switch via the Pi terminal, press Ctrl + C and it will abort the script, putting the UAV in LAND mode.

# Connect to the drone via serial port
print("Connecting to UAV CubeOrange via /dev/ttyAMA0...")
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
print("UAV Pi is successfully connected to the Cube Orange!\n")

# Log file path
# Note: Rerunning the program DOES NOT overwrite the past program run. Instead, it will continue to add to the .txt file
log_file = "LOOK_AT_ME_drone_stats.txt"

def log_drone_status(signum, frame):
    """
    ISR function that logs UAV telemetry data to a text file every 2 seconds.
    Captures mode, altitude, velocity, and GPS coordinates.
    """
    with open(log_file, "a") as f:
        f.write(f"Time: {time.strftime('%H:%M:%S')}\n")
        f.write(f"Mode: {vehicle.mode.name}\n")
        f.write(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m\n")
        f.write(f"Velocity: {vehicle.velocity}\n")
        f.write(f"GPS: {vehicle.location.global_frame}\n")
        f.write("-" * 30 + "\n")

# Set up a timer ISR to trigger `log_drone_status` every 2 seconds
signal.signal(signal.SIGALRM, log_drone_status)
signal.setitimer(signal.ITIMER_REAL, 2, 2)

# Note:
# signal.setitimer(signal.ITIMER_REAL, first_time, interval)
# first_time: delay (in seconds) before the 1st interrupt is called
# interval: recurring intervals (in seconds) for the interrupt

# Emergency kill switch - Lands the drone immediately when Ctrl + C is pressed
def emergency_land(signal, frame):
    print("\nKill switch activated! Landing immediately...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(2)  # Allow time for the mode to switch
    vehicle.close()
    os._exit(0)

signal.signal(signal.SIGINT, emergency_land)

# creates a MAVLink message to send velocity commands in the NED (North, East, Down) frame.
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """Send velocity command in NED frame."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED, # tell autopilot to interpret values in body-relative NED frame
        0b0000111111000111,                 # bitmask that tells it: use only velocity fields
        0, 0, 0,                            # position (not used, set to zero)
        velocity_x, velocity_y, velocity_z, # velocity
        0, 0, 0,                            # acceleration (not used, set to zero)
        0, 0                                # yaw and yaw rate (not used, set to zero
    )
    # send message for requested duration
    vehicle.send_mavlink(msg)
    time.sleep(1)
    vehicle.flush() # Ensure MAVLink command is sent

# Arm and take off sequence
print("Waiting for vehicle to initialize...")
while not vehicle.is_armable:
    print("Waiting for drone to become armable...")
    time.sleep(1)

print("Arming motors...")
vehicle.mode = VehicleMode("GUIDED")  # Set to GUIDED mode for autonomous control
vehicle.armed = True  # Command the UAV to arm

# Wait until the UAV is armed
while not vehicle.armed:
    print("Waiting for drone to arm...")
    time.sleep(1)

print("Taking off...")
vehicle.simple_takeoff(3)  # takeoff initally to 3 meters for a safe start
while vehicle.location.global_relative_frame.alt < 2.5:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m")
    time.sleep(1)

print(('Reached floor, now ascending at 5 mph to 20 feet...'))
# Monitor altitude until the UAV reaches the desired height
while True:
    target_altitude = 7  # Target altitude in meters (about 22 feet)
    current_altitude = vehicle.location.global_relative_frame.alt

    # In-flight safety check: ensure we're still in GUIDED mode
    if vehicle.mode.name != "GUIDED":
        print("Failsafe triggered or manual override detected. Initiating emergency land.")
        vehicle.mode = VehicleMode("LAND")
        break

    # Ensure the drone reaches at least 95% of the target altitude
    if current_altitude >= target_altitude * 0.95:
        print("Reached target altitude of 20 feet")
        break
        
    send_ned_velocity(0, 0, -2.3, 1)  # Ascend at ~5 mph
    print(f"Altitude: {current_altitude:.2f} m")
    time.sleep(1)

# Hover in place for a few seconds before proceeding
print("Hovering for 12 seconds...")
time.sleep(12)

# Land the drone after hovering
print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Allow time for landing
while vehicle.location.global_relative_frame.alt > 0.1:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m - Descending...")
    time.sleep(1)
print("Landed successfully!")

# Close the vehicle connection
print("Logging stopped, Closing vehicle connection...")
vehicle.close()
print("Mission Complete")
