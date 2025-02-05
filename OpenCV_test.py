import cv2
from picamera2 import Picamera2
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()
time.sleep(2)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
arucoParameters = cv2.aruco.DetectorParameters_create()

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
