"""
RAYTHEON CSE TEAM 2024-2025

PROGRAM TO RUN COMMUNICATION BETWEEN UAV AND UGV
"""

""" START OF IMPORTS """

# Usual Python imports
import socket

""" END OF IMPORTS """



""" START OF FUNCTIONS """

# UGV_PORT should be constant
def client_init(UGV_IP, UGV_PORT):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    
    while not connected:
        try:
            client.connect((UGV_IP, UGV_PORT))
            connected = True
        except Exception as e:
            pass
    
    print("UAV client is connected to UGV")

    return client

def send_msg(client, msg):
    # Send a message to UGV (in this case, the UAV's coordinates)
    client.send(msg.encode())

    # Get response from UGV if any
    response = client.recv(1024).decode()
    print(f"Response from UGV: {response}")

    # return response # In case any info is needed

""" END OF FUNCTIONS """
