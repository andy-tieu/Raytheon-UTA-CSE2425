from dronekit import connect, VehicleMode
import time
import signal
import os

# This script does the following:
# 1.) Connects to the Cube Orange via serial on /dev/serial0.
# 2.) Waits for the drone to be ready.
# 3.) Arms the motors.
# 4.) Takes off to 1 meter altitude.
# 5.) Waits until the altitude is reached.
# 6.) Hovers for 5 seconds.
# 7.) Logs UAV telemetry data to a file every 2 seconds using a timer ISR.
# 8.) Lands the drone.
# 9.) Closes the connection and exits.
#
# Note: This script needs GPS to function. UAV will not switch to guided mode without GPS Lock.
# To kill switch via the Pi terminal, press Ctrl + C and it will abort the script, putting the UAV in LAND mode.

# Connect to the drone via serial port
print("Connecting to UAV CubeOrange via /dev/serial0...")
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
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
target_altitude = 1  # Target altitude in meters
vehicle.simple_takeoff(target_altitude)  # Command takeoff

# Monitor altitude until the UAV reaches the desired height
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {current_altitude:.2f} m")
    
    # Ensure the drone reaches at least 95% of the target altitude
    if current_altitude >= target_altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Hover in place for a few seconds before proceeding
print("Hovering for 5 seconds...")
time.sleep(5)

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
