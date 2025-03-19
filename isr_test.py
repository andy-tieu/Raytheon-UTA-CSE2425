from dronekit import connect, VehicleMode
import time
import signal

# ISR (Interrupt Service Routine): Allows programs to be interrupted mid program execution
# This program will always document the drone's status every 2 seconds regardless of what line of code is executed

# Connect to the drone via serial port
print("Connecting to UAV CubeOrange via /dev/serial0...")
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
print("UAV Pi is successfully connected to the Cube Orange!\n")

# Log file path
log_file = "drone_status.txt"

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

# The Rest of the Program below can be replaced with the drone's automation and ArUco detection
# (IMPORTANT) Just make sure the vehicle.close() function is called (IMPORTANT)
print("Flight data is being logged every 2 secods")
try:
	while True:
		time.sleep(1)
except KeyboardInterrupt:
		print("Logging Stopped. Ending connection.")
		vehicle.close()
		print("Mission Complete")
