import socket
import time
from dronekit import connect, Command, VehicleMode, LocationGlobal
from pymavlink import mavutil
# make sure this module is uploaded to the UGV RPi
from vehicle_logging import log_comm_receive, log_delivery, log_end

# Connect to the UGV
vehicle = connect('/dev/serial0', baud=57600, wait_ready=True)
print("Preliminary Checks")
print("------------------------------------------------------")
print("UGV Pi is Connected to Pixhawk!\n")

# Set the servo to its initial position (1300)
vehicle.channels.overrides['8'] = 2000
print("Delivery Gate Up!\n")

def clear_mission():
    """
    Clear existing mission on the UGV.
    """
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()
    time.sleep(1)  # Allow time for mission clearing

def add_waypoint(lat, lon, alt):
    """
    Add a single waypoint to the mission.
    """
    cmds = vehicle.commands
    cmds.clear()
    cmds.add(
        Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
            lat, lon, alt
        )
    )
    cmds.upload()
    print(f"Waypoint added: Latitude={lat}, Longitude={lon}, Altitude={alt}")

# Socket Server Setup
def start_socket_server(host='0.0.0.0', port=12345):
    """
    Start a socket server to listen for waypoint data from the UAV.
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(1)
    print(f"Server listening on {host}:{port}\n")

    while True:
        print("Waiting for UAV connection...")
        conn, addr = server.accept()
        print("Connected to UAV at {addr}")
        print("------------------------------------------------------\n")
        # Clear the mission before adding new waypoints
        clear_mission()

        # Receive data
        data = conn.recv(1024).decode()
        if data:
            print(f"Delivery Zone coordinates received: {data}\n")
            log_comm_receive(data)  # Log the received message
            try:
                # Parse the received coordinates
                lat, lon, alt = map(float, data.split(","))
                add_waypoint(lat, lon, alt)

                conn.send("Waypoint added.".encode())

                # Switch to AUTO mode to begin mission execution
                print("\n")
                print("Switching to AUTO mode...")
                vehicle.mode = VehicleMode("AUTO") 

                while not vehicle.mode.name == "AUTO":
                    print("Waiting for mode change...")
                    time.sleep(1)
                print("Current mode:", vehicle.mode.name)

                # Ensure mission starts from the first waypoint
                vehicle.commands.next = 0

                # Wait for the mission to start
                while vehicle.commands.next == 0:
                    print("Waiting for mission start...")
                    time.sleep(1)
                #UAV Start Time Log
                print("Mission started!")
                print(f"Timestamp (Mission Start): {time.strftime('%Y-%m-%d %H:%M:%S')}")

                # Monitor mission completion
                while vehicle.mode.name == "AUTO":
                    if vehicle.commands.next == 1:  # Next command index reaches the last waypoint
                        print("Waypoint Reached!")
                        break
                    time.sleep(1)

                # Set the servo position once the mission ends
                print("Delivery Gate Opened!")
                vehicle.channels.overrides['8'] = 1000
                time.sleep(2)
                # Log the delivery event
                log_delivery(f"{lat},{lon}",{alt})

                # UGV End Time
                print(f"Timestamp (Mission Complete): {time.strftime('%Y-%m-%d %H:%M:%S')}")
                vehicle_logging.log_end()
            except ValueError:
                conn.send("Invalid coordinate format.".encode())
                print("Invalid data format received.")

            conn.close()

# Main script execution
if __name__ == "__main__":
    print("Clearing mission...\n")
    clear_mission()
    
    print("Starting socket server...")
    start_socket_server()

    vehicle.close()
