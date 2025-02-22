import cv2
from picamera2 import Picamera2
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()
time.sleep(2)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
arucoParameters = cv2.aruco.DetectorParameters_create()
dropzone = 3

try:
    while True:
        frame = picam2.capture_array()  # Capture frame
        
        if frame is None:
            print("Error: No frame output")
            continue
            
        time.sleep(0.1)

        # Turns the frame into grayscale to make it easier to identify markers
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Takes in the image, dictionary, and parameters to the first, second, and fifth parameters
        # Returns detected corners, id of the marker, and any conrner points of rejected markers
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParameters) 
        
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Convert to BGR

        if ids is not None:
            # Draws on top of the actual frame (since we convert to BGR, it will allow the drawing to work since otherwise it will error out)
            cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

            # TODO: 
            # TEST THE CENTERING AND HOW IT WOULD WORK IN THE AIR
            # FIX FOCUS (POSSIBLY CHANGE TO AUTOFOCUS MODE)
            # W/ DRONE, SEND COORDINATES OF WHERE WE ARE WHEN DETECTING AN ARUCO MARKER (DOESN'T HAVE TO BE 100% ACCURATE YET)

            # Idea of centering is to ensure when UAV is flying over a marker, it can reposition itself until the marker is aligned better
            # to then have the clear to send the coordinates to the UGV
            # marker_center_x = (corners[0][0][0] + corners[0][0][2]) / 2
            # marker_center_y = (corners[0][0][1] + corners[0][0][3]) / 2
            # print(f"X Center is: {marker_center_x} | Y Center is: {marker_center_y}")
            
            # frame_center_x = frame.shape[1] // 2
            # frame_center_y = frame.shape[0] // 2
            
            # Offset can be changed depending on frame size and how far away we are from the ArUco marker
            # This will need to be tested to have a good idea on what offset we should have to send accurate coordinates
            # offset_x = marker_center_x - frame_center_x
            # offset_y = marker_center_y - frame_center_y
            # print(f"X Offset is: {offset_x} | Y Offset is: {offset_y}")
            
            print(f"ArUco Marker with ID: {ids} detected")
            for i in range(len(ids)):
                if ids[i] == dropzone:
                    print("DropZone detected")
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
