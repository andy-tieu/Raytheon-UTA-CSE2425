# This program integrates md_send_integration.py with camera centering functionality and logging
# This program is designed to connect to the Cube Orange, arm the drone in POSHOLD MODE, before taking off on the waypoints mission in AUTO MODE
# Once the drone detects the correct ArUco Marker, it will switch to GUIDED MODE and center itself over the dropzone
# It will then send the current GPS coordinates to the UGV and return to launch point

# TODO:
# Program needs testing on camera centering
# Would be good if this program were redone using OOP, this could replace full_test.py

from dronekit import connect, VehicleMode, LocationGlobal
import time, socket
import cv2
from picamera2 import Picamera2
import vehicle_logging
import threading

"""
Arms vehicle
"""
def arm():
    print("Basic pre-arm checks")
    # Ensures user does not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Arming motors")
    # Copter should arm in poshold mode
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

#kill switch implementation --------------------------------------------
def emergency_land(signal, frame):
    """ Emergency landing function triggered by CTRL+C """
    print("\n Kill switch activated! Landing immediately...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.flush()  # Ensure MAVLink command is sent
    time.sleep(1)
    vehicle.close()
    os._exit(0)

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

def send_ned_velocity(vehicle, forward_speed, right_speed, down_speed, check_interval = 0.1):
    """ Move drone using velocity commands (m/s) for a specific duration """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # Target system, target component, coordinate frame
        0b0000111111000111,  # Bitmask: Only control velocities
        0, 0, 0,  # Position (ignored)
        velocity_x, velocity_y, velocity_z,  # Velocity (m/s)
        0, 0, 0,  # Acceleration (ignored)
        0, 0  # Yaw, yaw rate (ignored)
    )
    
    for _ in range(duration * 10):  # Send command every 100ms for 'duration' seconds
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def move_to_correct_position(dx, dy):
    """ Moves the drone based on ArUco marker offsets using velocity commands """
    # The offset values are multiplied by an arbitrary to get a low speed value, feel free to reduce the decimal number as needed
    velocity_x = -dy * 0.05  # Forward/backward
    velocity_y = -dx * 0.05  # Left/right

    print(f"Adjusting position: dx={dx}, dy={dy}")
    print(f"Sending velocity command: vx={velocity_x}, vy={velocity_y}")

    # Move the drone
    send_ned_velocity(vehicle, velocity_x, velocity_y, 0, 2)

    # Stop movement after duration
    send_ned_velocity(vehicle, 0, 0, 0, 1)  # Stop movement

# This program detects ArUco markers using a Raspberry Pi camera and OpenCV
# It distinguishes a specific marker separate from others as the dropzone
# After which it will determine if the drone is centered over the dropzone and send coordinates to the UGV
def camera_init():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    # Using smallest dictionary since there are only markers from ID 1 to ID 5
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParameters = cv2.aruco.DetectorParameters_create()
    
    # The dropzone variable can be changed to any ID; our current tests use marker 3 as the designated dropzone
    dropzone = 2
    
    detect_marker(picam2, arucoDict, arucoParameters, dropzone)

def detect_marker(picam2, arucoDict, arucoParameters, dropzone):
    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            
            if frame is None:
                print("Error: No frame output")
                continue

            time.sleep(0.25)

            # Turns the frame into grayscale to make it easier to identify markers
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # Takes in the image, dictionary, and parameters to the first, second, and fifth parameters
            # Returns detected corners, id of the marker, and any conrner points of rejected markers
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParameters)

            # Convert to BGR, otherwise the display will have issues
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # This extracts the height and width of the actual camera frame we currently have
            # Since the frame is always a square or rectangle, dividing by 2 will get the center of either axis
            height, width, _ = frame.shape
            center_x, center_y = width // 2, height // 2

            # Define error margin square (e.g., 25x25 pixels)
            # Margin will need testing since it will greatly depend on how high up we are
            margin = 25

            # Draws the rectangle onto our current frame; corners are calculated
            cv2.rectangle(frame_bgr, (center_x - margin, center_y - margin), (center_x + margin, center_y + margin), (0, 255, 0), 2)

            if ids is not None:
                # Draws on top of the actual frame (since we convert to BGR, it will allow the drawing to work since otherwise it will error out)
                cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

                # TODO: 
                # TEST THE CENTERING AND HOW IT WOULD WORK IN THE AIR
                # ADD FUNCTIONALITY TO ALLOW DRONE TO MOVE AND CENTER ITSELF BASED ON OFFSET VALUES
                # FIX FOCUS (POSSIBLY CHANGE TO AUTOFOCUS MODE)
                # W/ DRONE, SEND COORDINATES OF WHERE WE ARE WHEN DETECTING THE DROPZONE MARKER
                

                print(f"ArUco Marker with ID: {ids} detected")
                # Log the discovery of the ArUco marker
                vehicle_logging.log_aruco_discovery(ids)
                for i in range(len(ids)):
                    # Extract corner points
                    c = corners[i][0]

                    if ids[i] == dropzone:

                        print("DropZone detected")
                        log_dropzone_discovery(ids[i]) # ADAM LOGGING
                        print("Entering GUIDED mode")
                        vehicle.flush()


                    while vehicle.mode.name != "GUIDED":
                            print("Waiting for mode change...")
                            time.sleep(1)
                    print("Drone is now in GUIDED mode and hovering.")

                        # We only care about the location of the drop zone, so all centering code is located within it
                        marker_center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                        cv2.circle(frame_bgr, marker_center, 5, (0, 0, 255), -1)
                        dx, dy = marker_center[0] - center_x, marker_center[1] - center_y

                       # if (center_x - margin <= marker_center[0] <= center_x + margin) and (center_y - margin <= marker_center[1] <= center_y + margin):
                        if ids[i] == dropzone:
                            
                            print("Marker centered!")
                            # Add code to send coordinates here

                            # Get current location
                            latitude = vehicle.location.global_frame.lat
                            longitude = vehicle.location.global_frame.lon
                            altitude = vehicle.location.global_frame.alt

                            print(f"Location: Lat={latitude}, Lon={longitude}, Alt={altitude}")

                            # Prepare data to send
                            coordinates_str = f"{latitude},{longitude},{altitude}"

                            # log coordinates
                            vehicle_logging.log_dropzone_location(ids[i], coordinates_str)
                            # Send the coordinates to the UGV
                            try:
                                client.send(coordinates_str.encode())

                                response = client.recv(1024).decode()
                                print(f"Response from UGV: {response}")
                            except Exception as e:
                                print(f"Error sending coordinates: {e}")
                            finally:
                                vehicle_logging.log_comm_transmit("Coordinates sent successfully to UGV")
                                client.close()
                        else:
                            vehicle.mode = VehicleMode("GUIDED")
                            print("Moving...")
                            # Add code to move drone closer to being centered
                            # Issue to address: drone may be too high and give inaccurate coordinates;
                            # need to factor in height adjustments as well (we can have drone lower to a certain altitude or
                            # lower until no longer centered and recenter before continuing to lower until specified altitude)
                            # added code (in progress)
                            # Marker is not centered, adjust UAV position
                            print("Moving...")
                            forward_speed = -0.5 if dy > 0 else (0.5 if dy < 0 else 0)  # Move forward/backward
                            right_speed = 0.5 if dx > 0 else (-0.5 if dx < 0 else 0)    # Move left/right
                            down_speed = 0  # No vertical adjustment for now; can be added if needed

                            # Send velocity commands to adjust UAV position
                            send_ned_velocity(vehicle, forward_speed, right_speed, down_speed, check_interval=0.1)

                            print(f"Adjusting position: forward_speed={forward_speed}, right_speed={right_speed}, down_speed={down_speed}")
                            move_to_correct_position(dx, dy)
                    else:
                        print("Non-DropZone detected")
            else:
                print('No markers detected')

            # Displays the frames on the camera feed, so the user can see that a marker has been identified
            cv2.imshow("ArUco Marker Detection", frame_bgr)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Program interrupted!")
                break
    except KeyboardInterrupt:
        print("Program interrupted!")

    finally:
        # Release the capture and close any OpenCV windows
        # Return to launch
        print("Returning to launch point...")
        vehicle.mode = VehicleMode("RTL")
        vehicle.flush()
        cv2.destroyAllWindows()
        picam2.stop()
        

"""
connection_string = 'tcp:127.0.0.1:57600'  
print('Connecting to vehicle on: %s' % connection_string)

vehicle = connect(connection_string, wait_ready = True)
"""
if __name__ == "__main__":
    # Connect to the UAV
    print("Connecting to UAV CubeOrange via /ddev/ttyAMA0...")
    vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    print("UAV Pi is successfully connected to the Cube Orange!\n")

    # UGV IP and port
    # UGV's IP on PortablWiFi: 192.168.1.21
    # UGV's IP on Hottest Spot: 192.168.1.25
    # UGV's IP on Makerspace: 192.168.1.56
    ugv_ip = '192.168.1.56' # Replace with the UGV's IP address
    ugv_port = 12345

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ugv_ip, ugv_port))
    arm()
    # Start the listener in a separate thread
    kill_thread = threading.Thread(target=listen_for_kill, daemon=True)
    kill_thread.start()
    camera_init()

