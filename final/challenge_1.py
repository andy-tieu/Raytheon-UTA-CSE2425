"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO RUN CAMERA AND ARUCO MARKER DETECTION
"""

""" START OF IMPORTS """

# Dependencies; need to be installed
import cv2 # opencv
from picamera2 import Picamera2 # for RPi camera module use

# Usual Python imports
import time

# Other imports
from drone_comm import *
from drone_movement import *
from vehicle_logging import *

""" END OF IMPORTS """



""" START OF FUNCTIONS """

class ArucoDetection:
    def __init__(self, vehicle, DROPZONE, gui):

        # Vehicle object to get current latitude and longitude
        self._vehicle = vehicle

        # ArUco Dictionary init
        # Using smallest dictionary since there are only markers from ID 1 to ID 5
        self._arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self._parameters = cv2.aruco.DetectorParameters_create()

        # ArUco info
        self.DROPZONE = DROPZONE

        # Camera init
        self._picam2 = Picamera2()
        self._config = self._picam2.create_preview_configuration()
        self._picam2.configure(self._config)
        self._picam2.start()
        self._gui = gui

        # Variables
        self._centered = False
        self._timeout = 0
        
        # Set to track logged ArUco IDs
        self._logged_ids = []
        self._dropzoneflag = False

    def stopCam(self):
        # Turn off camera when needed
        self._picam2.stop()

    # Function to detect correct dropzone ArUco marker
    def detect(self):
        while not self._centered:
            # Capture frame
            frame = self._picam2.capture_array()
            
            if frame is None:
                print("Error: No frame output")
                continue

            time.sleep(0.2)

            # Turns the frame into grayscale to make it easier to identify markers
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # Takes in the image, dictionary, and parameters to the first, second, and fifth parameters
            # Returns detected corners, id of the marker, and any conrner points of rejected markers
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, self._arucoDict, parameters=self._parameters)    

            # Convert to BGR, otherwise the display will have issues
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # This extracts the height and width of the actual camera frame we currently have
            # Since the frame is always a square or rectangle, dividing by 2 will get the center of either axis
            height, width, _ = frame.shape
            center_x, center_y = width // 2, height // 2

            # Define error margin square (e.g., 25x25 pixels)
            # Margin will need adjusting depending on drone's altitude
            margin = 75

            # Draws the rectangle onto our current frame; corners are calculated
            cv2.rectangle(frame_bgr, (center_x - margin, center_y - margin), (center_x + margin, center_y + margin), (0, 255, 0), 2)

            if ids is not None:
                # Draws on top of the actual frame (since we convert to BGR, it will allow the drawing to work since otherwise it will error out)
                cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)
                
                for i in range(len(ids)):
                    # Extract corner points
                    c = corners[i][0]

                    if ids[i] == self.DROPZONE:
                        self._vehicle.mode = VehicleMode("GUIDED")
                        # self._vehicle.flush() # not technically needed since switching modes goes through instantly

                        while self._vehicle.mode.name != "GUIDED":
                            print("Waiting for mode change...")
                            time.sleep(1)

                        print("Drone is now in GUIDED mode and hovering.")
                        if ids[i] not in self._logged_ids:
                            log_dropzone_discovery(ids[i]) # ADAM LOGGING
                            print("DropZone detected")

                        # We only care about the location of the drop zone, so all centering code is located within it
                        marker_center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                        cv2.circle(frame_bgr, marker_center, 5, (0, 0, 255), -1)
                        dx, dy = marker_center[0] - center_x, marker_center[1] - center_y                        

                        if (center_x - margin <= marker_center[0] <= center_x + margin) and (center_y - margin <= marker_center[1] <= center_y + margin):
                            print("Marker centered!")

                            # Get current location
                            latitude = self._vehicle.location.global_frame.lat
                            longitude = self._vehicle.location.global_frame.lon
                            altitude = self._vehicle.location.global_frame.alt

                            print(f"Location: Lat: {latitude}\t Lon: {longitude}\t Alt: {altitude}")

                            # Prepare data to send
                            coordinates_str = f"{latitude},{longitude},{altitude}"
                            log_comm_transmit(coordinates_str) # ADAM LOGGING
                            log_dropzone_location(ids[i], coordinates_str) # ADAM LOGGING

                            # After the marker is centered and coordinates are sent, the UAV is returned to launch point
                            # This portion of code can also be replaced with landing the drone at a specific landing zone
                            print("Returning to launch point...")
                            self._vehicle.mode = VehicleMode("RTL")
                            # self._vehicle.flush()

                            while self._vehicle.mode.name != "RTL":
                                print("Waiting for mode change...")
                                time.sleep(1)

                            print("Drone is now in RTL mode and returning.")

                            # This should prevent drone from continuing program
                            self.stopCam()
                            self._centered = True

                            # Wait until the drone disarms after RTL (meaning it has landed)
                            while vehicle.armed:
                                print("Waiting for landing...")
                                time.sleep(1)

                            print("Drone has landed and disarmed.")
                            log_end()

                        else:
                            # Marker is not centered, adjust UAV position
                            print("Moving...")
                            forward_speed = -0.1 if dy > 0 else (0.1 if dy < 0 else 0)  # Move forward/backward
                            right_speed = 0.1 if dx > 0 else (-0.1 if dx < 0 else 0)    # Move left/right
                            down_speed = 0  # No vertical adjustment for now; can be added if needed

                            # Send velocity commands to adjust UAV position; movement is based on drone's direction
                            send_body_velocity(self._vehicle, forward_speed, right_speed, down_speed, check_interval=0.01)
                            print(f"Adjusting position: forward_speed={forward_speed}, right_speed={right_speed}, down_speed={down_speed}")
                            
                            time.sleep(1)
                            self._timeout += 1
                            if self._timeout == 15:
                                print("15 seconds exceeded of attempting to move, sending current location!")
                                
                                # Get current location
                                latitude = self._vehicle.location.global_frame.lat
                                longitude = self._vehicle.location.global_frame.lon
                                altitude = self._vehicle.location.global_frame.alt

                                print(f"Location: Lat: {latitude}\t Lon: {longitude}\t Alt: {altitude}")

                                # Prepare data to send
                                coordinates_str = f"{latitude},{longitude},{altitude}"
                                log_comm_transmit(coordinates_str) # ADAM LOGGING
                                log_dropzone_location(ids[i], coordinates_str) # ADAM LOGGING

                                # After the marker is centered and coordinates are sent, the UAV is returned to launch point
                                # This portion of code can also be replaced with landing the drone at a specific landing zone
                                print("Returning to launch point...")
                                self._vehicle.mode = VehicleMode("RTL")
                                # self._vehicle.flush()

                                while self._vehicle.mode.name != "RTL":
                                    print("Waiting for mode change...")
                                    time.sleep(1)

                                print("Drone is now in RTL mode and returning.")

                                # This should prevent drone from continuing program
                                self.stopCam()
                                self._centered = True
                                
                                # Wait until the drone disarms after RTL (meaning it has landed)
                                while vehicle.armed:
                                    print("Waiting for landing...")
                                    time.sleep(1)

                                print("Drone has landed and disarmed.")
                                log_end()

                    else:
                        if ids[i] not in self._logged_ids:
                            self._logged_ids.append(ids[i])
                            log_aruco_discovery(ids[i]) # ADAM LOGGING
                            print(f"ArUco Marker with ID: {ids[i]} detected")

                        print("Non-DropZone detected")
            #else:
                # This can be commented out since it will flood terminal; used for testing
                #print("No markers detected")

            # If gui is set to true, camera feed will show up in a new window
            if self._gui:
                # Displays the frame
                cv2.imshow("ArUco Marker Detection", frame_bgr)

                # Use 'q' to quit or ctrl + c in terminal
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self._picam2.stop()
                    cv2.destroyAllWindows()
                    break
        
        self.stopCam()

""" END OF FUNCTIONS """



""" START OF MAIN """

if __name__ == "__main__":

    vehicle = drone_connect('/dev/ttyAMA0')

    # Put UGV_IP address here to be used
    # UGV's IP on PortablWiFi: 192.168.1.21
    # UGV's IP on Hottest Spot: 192.168.1.25
    # UGV's IP on Makerspace: 192.168.1.56
    UGV_IP = '192.168.1.25' # Replace with the UGV's IP address
    UGV_PORT = 12345 # Constant port used for connection; ensure UGV is using it

    # Change dropzone ID here to whichever is given at competition
    DROPZONE = 4

    # Create camera object
    gui = False # Change this if you want video feed or not
    aruco_detection = ArucoDetection(vehicle, DROPZONE, gui)

    #signal.signal(signal.SIGINT, emergency_land)

    #kill_thread = threading.Thread(target=listen_for_kill, daemon=True)
    #kill_thread.start()

    # Start detection
    log_start()
    aruco_detection.detect()

""" END OF MAIN """
