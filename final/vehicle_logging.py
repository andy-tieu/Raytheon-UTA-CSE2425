"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO LOG UAV AND UGV DATA
"""

""" START OF IMPORTS """

# Basic imports
import time
import signal

""" END OF IMPORTS """



""" START OF GLOBALS """

# Global variables
filename = "uav_status.txt"     # Log file name
                                # CHANGE FILE NAME FOR UAV or UVG LOGGING
debug = True                    # Debug mode: For verifying when logging occurs, turn off when not debugging

""" END OF GLOBALS """



""" START OF FUNCTIONS """

# "Appendix C" – Logging Requirements:
# UAV Logs: UAV Start Time, ArUco discovery, Communication between UxV’s, Location of the ArUco, UAV End time
# UGV Logs: UxV Start Time, UxV receipt of ArUco location, UxV Delivery Time, Communication between UxV’s, UxV End time

def log_start():
    # Open the log file in write mode to start logging
    # THE PREVIOUS LOG FILE WILL BE OVERWRITTEN
    with open(filename, "w") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tStart Time\n")
    
    if debug:
        print("Logging started.")

def log_aruco_discovery(id):
    # Append the discovery time of any ArUco markers to log file
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tArUco ID {id} discovered\n")

    if debug:
        print(f"ArUco ID {id} discovered.")

def log_dropzone_discovery(id):
    # Append the discovery time of the drop zone ArUco marker to the log file
    # This function should only be called when the drop zone ArUco marker is discovered
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tDrop Zone ArUco ID {id} discovered\n")
    
    if debug:
        print(f"Drop Zone ArUco ID {id} discovered.")

def log_dropzone_location(id, coordinates):
    # Append the location of the ArUco marker to the log file
    # This function should only be called when the drop zone ArUco marker is discovered AND centered in the camera view
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tDrop Zone ArUco ID {id} is at {coordinates}\n")
    
    if debug:
        print(f"Drop Zone ArUco ID {id} is at {coordinates}.")

def log_comm_transmit(msg):
    # Append the transmission of a message to UGV to the log file
    # This function should be called when the UAV transmits a message to the UGV
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tMessage Transmitted: {msg}\n")
    
    if debug:
        print(f"Message Transmitted: {msg}")

def log_comm_receive(msg):
    # Append the reception of a message from UAV to the log file
    # This function should be called when the UGV receives a message from the UAV
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tMessage Received: {msg}\n")
    
    if debug:
        print(f"Message Received: {msg}")

def log_delivery(coordinates):
    # Append the delivery time of the payload to the log file
    # This function should be called when the UGV delivers the payload to the drop zone
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tPayload Delivered at {coordinates}\n")
    
    if debug:
        print(f"Payload Delivered at {coordinates}.")

def log_end():
    # Append the end time of the vehicle to the log file
    # This function should be called when the vehicle has completed its mission
    with open(filename, "a") as f:
        f.write(f"{time.strftime('%H:%M:%S')}:\tEnd Time\n")

    if debug:
        print("Logging ended.")

""" END OF FUNCTIONS """



""" START OF NOTES """

# This script is designed to log UAV drone data based on isr_test.py in the "tests" folder and other libraries in the "final" folder.

""" END OF NOTES """