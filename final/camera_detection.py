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
    def __init__(self, vehicle, DROPZONE, gui, client):

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

        # Socket client to send coordinates when needed
        self._client = client

    def stopCam(self):
        # Turn off camera when needed
        self._picam2.stop()

    # Function to detect correct dropzone ArUco marker
    def detect(self):
        while True:
            # Capture frame
            frame = self._picam2.capture_array()
            
            if frame is None:
                print("Error: No frame output")
                continue

            time.sleep(0.25)

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
            # Margin will need testing since it will greatly depend on how high up we are
            margin = 25

            # Draws the rectangle onto our current frame; corners are calculated
            cv2.rectangle(frame_bgr, (center_x - margin, center_y - margin), (center_x + margin, center_y + margin), (0, 255, 0), 2)

            if ids is not None:
                # Draws on top of the actual frame (since we convert to BGR, it will allow the drawing to work since otherwise it will error out)
                cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

                log_aruco_discovery(ids) # ADAM LOGGING
                print(f"ArUco Marker with ID: {ids} detected")
                for i in range(len(ids)):
                    # Extract corner points
                    c = corners[i][0]

                    if ids[i] == self.DROPZONE:
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
                            msg = "test"
                            self._client.send_msg(self._client, msg) # Change out msg to coordinates_str when needed
                            log_comm_transmit(msg) # ADAM LOGGING
                            log_dropzone_location(ids[i], coordinates_str) # ADAM LOGGING
                        else:
                            print("Moving...")
                            # TODO: 
                            # ADD FUNCTIONALITY TO ALLOW DRONE TO MOVE AND CENTER ITSELF BASED ON OFFSET VALUES
                            # TEST THE CENTERING AND HOW IT WOULD WORK IN THE AIR

                            # Add code to move drone closer to being centered
                            # Issue to address: drone may be too high and give inaccurate coordinates;
                            # need to factor in height adjustments as well (we can have drone lower to a certain altitude or
                            # lower until no longer centered and recenter before continuing to lower until specified altitude)
                    else:
                        print("Non-DropZone detected")
            else:
                print("No markers detected")

            # If gui is set to true, camera feed will show up in a new window
            if self.gui:
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

    # Change dropzone ID here to whichever is given at competition
    DROPZONE = 3

    # Create camera object
    gui = False # Change this if you want video feed or not
    aruco_detection = ArucoDetection(DROPZONE, gui)

    # Start detection
    aruco_detection.detect()

""" END OF MAIN """