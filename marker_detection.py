# This program detects ArUco markers using a Raspberry Pi camera and OpenCV
# It distinguishes a specific marker separate from others as the dropzone
# After which it will determine if the drone is centered over the dropzone and send coordinates to the UGV

import time

import cv2
from picamera2 import Picamera2


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
    dropzone = 3
    
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
                for i in range(len(ids)):
                    # Extract corner points
                    c = corners[i][0]

                    if ids[i] == dropzone:
                        print("DropZone detected")

                        # We only care about the location of the drop zone, so all centering code is located within it
                        marker_center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                        cv2.circle(frame_bgr, marker_center, 5, (0, 0, 255), -1)
                        dx, dy = marker_center[0] - center_x, marker_center[1] - center_y                        

                        if (center_x - margin <= marker_center[0] <= center_x + margin) and (center_y - margin <= marker_center[1] <= center_y + margin):
                            print("Marker centered!")
                            # Add code to send coordinates here
                        else:
                            print("Moving...")
                            # Add code to move drone closer to being centered
                            # Issue to address: drone may be too high and give inaccurate coordinates;
                            # need to factor in height adjustments as well (we can have drone lower to a certain altitude or
                            # lower until no longer centered and recenter before continuing to lower until specified altitude)
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
        cv2.destroyAllWindows()
        picam2.stop()

if __name__ == "__main__":
    camera_init()