# Raytheon CSE 2024-2025 Senior Design Project
  Repository for all the code developed for the Raytheon Drone Showcase Competition 2024-2025 by the University of Texas at Arlington's Computer Science and Engineering team.

## Introduction
  Welcome to the GitHub repository of the UTA CSE team! If you are looking to use and run the programs in this repository, please look through all the sections related to set-up and the individual programs themselves to learn how to use them with ease.
> [!IMPORTANT]
> All programs were written and ran on a device with a Linux Operating System using Python Version 3.x.x. 

## Content
  * [Setting up the Raspberry Pi](#initial-set-up)
  * [Simulated drone arming/disarming program](#sim_connectpy)
  * [Indoor flight testing program](#flight_testpy)
  * [Client communication program](#clientpy)
  * [Computer vision program](#marker_detectionpy)

## Initial Set-Up
  Before attempting to run anything, please ensure that your Raspberry Pi is properly set up to ensure there are no compatibility issues that arise during the use of programs.  
> [!NOTE]
> All code has been developed on a **Raspberry Pi 4B** and other Raspberry Pi devices (i.e. the RPi 3, other versions of 4, 5, and etc.) could have compatibility issues with the programs and it is suggested to use the same Raspberry Pi if possible.

  The Raspberry Pi Operating System (OS) used is the 64-bit version that is recommended when utilizing the Raspberry Pi Imager, which can be installed here: [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Please ensure the microSD card is fully loaded by the imager and then insert it into the Raspberry Pi device itself; make sure to have a mouse, keyboard, and monitor hooked up to the Raspberry Pi to finish the operating system set-up once power is turned on.  
  
  Once the Raspberry Pi Operating System has been installed successfully, ensure you are connected to a network and perform a full upgrade on the device by running the following commands in the terminal:
  ```
  sudo apt update
  sudo apt full-upgrade
  ```
  For the camera used on the drone, the Raspberry Pi Camera Module is used. 

## sim_connect.py
  [insert information here]

## flight_test.py
  [insert information here]

## client.py
  The purpose of the client.py program is to allow peer-to-peer communication between the UAV and UxV drone. The UAV drone acts as the client and sends data to the UxV drone (acts as server) to accept and read. In this project, the UxV has an on-board router that acts as the server of the drone and allows for long range connection to allow communication to remain continuous on the field. To run the program, the connection must be edited to match the address and port numbers in the server program to allow for connection to occur. You can change these values by editing this section of the program:
  ```python
  def client_socket_init():
    # Server Pi's IP or Router IP
    host = '192.168.1.1'
    # Ensure port number is the same as the server's
    port = 12345
  ```

## marker_detection.py
  To run this program, ensure that the Raspberry Pi is updated to the latest version, and that the Pi camera module is connected to the Raspberry Pi via a ribbon cable.  
> [!IMPORTANT]
> The config.txt file in /boot/firmware/ **MUST** be edited if you are using the same camera as us. To do this, change the line ```camera_autodetect=1``` to ```camera_autodetect=0``` and also add the line ```dtoverlay=imx708``` in the section labeled as [all] at the very bottom of the file. You can access this directory by running ```cd /boot/firmware/``` or directly accessing and editing the file with ```sudo nano /boot/firmware/config.txt```.

> [!NOTE]
> If you are using a different camera than us, please follow the instructions given with the camera package instead as they could be different.

  Once the config.txt file is properly edited, install the latest version of OpenCV by running these commands in the terminal:
  ```
  sudo apt install -y python3-opencv
  sudo apt install -y opencv-data
  ```
  Once everything is installed properly, the program can be run by either terminal command or using a code editor on the Raspbery Pi such as Geany. If any editing is needed for the camera use, you can refer to the [Picamera2 Library](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf) for help regarding camera configurations.
