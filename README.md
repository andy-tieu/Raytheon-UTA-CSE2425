# Raytheon CSE 2024-2025 Senior Design Project
  Repository for all the code developed for the Raytheon Drone Showcase Competition 2024-2025 by the University of Texas at Arlington's Computer Science and Engineering team.

## Introduction

## Before Running Programs
  Before attempting to run anything, please ensure that your Raspberry Pi is properly set up to ensure there are no compatibility issues that arise during program runs. All code has been developed on a Raspberry Pi 4B and other Raspberry Pi devices (i.e. the RPi 3 or 5) could have compatibility issues with some programs. The Raspberry Pi Operating System (OS) used is the 64-bit version that is recommended when utilizing the Raspberry Pi Imager, which can be installed here: [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Please ensure the microSD card is fully loaded by the imager and then inserted into the Raspberry Pi device itself to allow it to finish set-up.  
  Once the Raspberry Pi OS has been installed and ran, ensure you are connected to a network and perform a full upgrade on the device by running the commands `sudo apt update` followed by `sudo apt full-upgrade`. 

## OpenCV_test.py
  To run this program, ensure that the Raspberry Pi is updated to latest version. (New RPi OS's should have mostly everything installed automatically)
Ensure that the Pi camera module is connected to the Raspberry Pi and the config.txt file in /boot/firmware/ is edited to include the camera_autodetect=0 and dtoverlay=imx708 in the section marked as [all].
Install the latest version of OpenCV by running the command 'sudo apt install -y python3-opencv' and 'sudo apt install -y opencv-data'.
Once everything is set up properly, the program can be run by either terminal or a code editor.
