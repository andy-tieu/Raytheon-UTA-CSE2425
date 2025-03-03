# Basic client program; will be utilized in conjunction with camera program to send coordinates
# once the UAV drone and marker are aligned with each other

import socket

def client_socket_init():
    # Server Pi's IP or Router IP
    host = '192.168.1.1'
    # Ensure port number is the same as the server's
    port = 12345

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    print(f"Connected to server!")

    return client_socket

def send_msg(client_socket):
    # TODO:
    # GET THE HOST IP OF UGV PI/ROUTER
    # FIGURE OUT WHAT IS SENT TO THE UGV (AND SEND THAT INFO CORRECTLY FORMATTED)
    # IMPLEMENT MESSAGE SENDING WITHIN CAMERA PROGRAM
    # ENSURE THERE IS NO INDEFINITE WAITING IN PROGRAMS

    while True:
        message = input("Enter message: ")
        if message == 'exit':
            break
        client_socket.send(message.encode())
        data = client_socket.recv(1024).decode()
        print(f"Received: {data}")

    client_socket.close()

if __name__ == "__main__":
    client_socket = client_socket_init()
    send_msg(client_socket)