"""
This is the beginning of the program using everything
to run the autonomous flying, detection, and communication
on the UAV drone.
"""

"""
TO-DO LIST HERE!!!:

- IF THINGS ARE ADDED, DEFINE FUNCTIONS FOR THEM TO MAKE EASIER USE
- ADD GOOD COMMENTS PLEASE; VERY IMPORTANT
- ADD THE REST OF THE CAMERA DETECTION CODE
- ADD CODE TO CONNECT TO THE UAV (CUBE ORANGE)
- ADD KILLSWITCH CODE TO MAKE SURE LANDING IS SAFE
- ADD CODE TO ARM/DISARM
- ADD CODE TO SEND GPS COORDINATES OF THE UAV
- ADD IMPORTS (AS NEEDED)
- ADD FLYING CAPABILITIES (IF POSSIBLE RN)
"""



""" BEGINNING OF IMPORTS """

from camera_detection import *
from drone_comm import *

""" END OF IMPORTS """



""" START OF FUNCTIONS """



""" END OF FUNCTIONS """



""" START OF MAIN """

if __name__ == "__main__":

    # Put UGV_IP address here to be used
    # UGV's IP on PortablWiFi: 192.168.1.21
    # UGV's IP on Hottest Spot: 192.168.1.25
    # UGV's IP on Makerspace: 192.168.1.56
    UGV_IP = '192.168.1.56' # Replace with the UGV's IP address
    UGV_PORT = 12345 # Constant port used for connection; ensure UGV is using it
    
    # Change dropzone ID here to whichever is given at competition
    DROPZONE = 3

    # Create client object
    client = client_init(UGV_IP, UGV_PORT)

    # Create camera object
    gui = False # Change this if you want video feed or not
    aruco_detection = ArucoDetection(DROPZONE, gui, client)

    # Start detection
    aruco_detection.detect()

""" END OF MAIN """