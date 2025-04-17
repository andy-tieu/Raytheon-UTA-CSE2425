"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO RUN RAYTHEON COMPETITION CHALLENGE(S)
"""

""" BEGINNING OF IMPORTS """

from camera_detection import *
from drone_comm import *
from drone_movement import *
from vehicle_logging import *

""" END OF IMPORTS """



""" START OF FUNCTIONS """

class SearchRelay:
    def __init__(self, vehicle, waypoints, DROPZONE, client):
        
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.DROPZONE = DROPZONE
        self.client = client

    def search(self):
        waypoint_list = self.waypoints

        # Create camera object
        gui = False # Change this if you want video feed or not
        aruco_detection = ArucoDetection(DROPZONE, gui, client)



        # This section of code is temporarily commented out, the UAV will execute waypoint missions (AUTO mode set in main)
        # for waypoint in waypoint_list:
        #     target_location = LocationGlobalRelative(waypoint[0], waypoint[1], waypoint[2])
        #     self.vehicle.simple_goto(target_location)

        #     while self.vehicle.mode.name == "GUIDED":
        #         print("Vehicle is in GUIDED mode!")

                # TODO:
                # ADD CODE HERE TO PERFORM MOVEMENT AS NEEDED
                # POSSIBLY USE A SEPARATE PROGRAM CONTAINING NECESSARY DRONE MOVEMENT FUNCTIONS
                # POSSIBLE ISSUE: CAMERA LOOPS INDEFINITELY AND DRONE CANNOT MOVE TO NEXT WAYPOINT
                
                


""" END OF FUNCTIONS """

debug = True # Debug mode for this script

""" START OF MAIN """

if __name__ == "__main__":

    # TODO:
    # Figure out how to get waypoints from Mission Planner here

    # Connect to UAV drone
    vehicle = drone_connect('/dev/ttyAMA0')

    # Put UGV_IP address here to be used
    # UGV's IP on PortablWiFi: 192.168.1.21
    # UGV's IP on Hottest Spot: 192.168.1.25
    # UGV's IP on Makerspace: 192.168.1.56
    
    UGV_IP = '192.168.1.56' # Replace with the UGV's IP address
    UGV_PORT = 12345 # Constant port used for connection; ensure UGV is using it
    
    # Change dropzone ID here to whichever is given at competition
    DROPZONE = 3

    # Start logging
    log_start() # ADAM LOGGING

    # We can use some test waypoints here
    waypoints = np.loadtxt("test.waypoints", delimiter = "\t", skiprows = 2)

    # Set UAV to AUTO mode to execute waypoint mission (snake search pattern)
    vehicle.mode = VehicleMode("AUTO")

    # Create client object
    client = client_init(UGV_IP, UGV_PORT)

    # Start the search pattern for drop zone
    search = SearchRelay(vehicle, waypoints, DROPZONE, client)


""" END OF MAIN """
