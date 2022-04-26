# Quadcopter Drone
## General description
[**Parts Used**](https://docs.google.com/document/d/12gUURExo5_172K4eFv0Nk9-rLC2fsNIIvI8Z3eGtbUg/edit?usp=sharing)

The code portion of the project is divided into two modules (reinforcement learning not complete yet):
* **DroneFirmware:** contains the C code that is compiled and uploaded to the Arduino to run the drone. Handles transceiver communication, individual control of each motor, interfaces with IMU to determine orientation, and stabilization using PID algorithms
* **SerialRemote:** Python code that handles user input on a computer (supports keyboard or joystick) and interfaces with the transceiver connected to a USB to Serial module to send commands to the drone. These commands include information about thurst, pitch, roll, yaw, etc.

## Requirements
**SerialRemote:** requirements can be installed by running the following with Python 3 installed (file found in the SerialRemote directory):

```pip install -r requirements.txt```

**DroneFirmware:** the following third party libraries are included in the Arduino sketch (not including standard Arduino libraries):
* Adafruit 10DOF
* Adafruit Unified Sensor
* Adafruit BMP085
* Adafruit LSM303
* Adafruit L3GD20
* PID_v2 by Brett Beauregard

## Images/Videos
### ESCs, motors, PDB
![image](https://user-images.githubusercontent.com/28303167/165224908-aa68e1cc-0a8c-4ff2-9d49-1e29de9f8b67.png)
![image](https://user-images.githubusercontent.com/28303167/165225022-3d7ae702-9e3b-43f5-872b-e590ae933ba6.png)
![image](https://user-images.githubusercontent.com/28303167/165225077-0014c3f7-4806-4a86-bac3-2c4e6e1582f1.png)

### Transceiver module
![image](https://user-images.githubusercontent.com/28303167/165226419-1afa68c2-d161-401f-b1ec-1f0acd09c3af.png)

### Setup for testing
Drone is attached to weights to test correct control of motors. Temporary Arduino Uno is used instead of Arduino Nano in final result.
![image](https://user-images.githubusercontent.com/28303167/165225133-c912586e-ef9b-4b57-9399-310702f9127e.png)

### Stationary flight test with weight
https://user-images.githubusercontent.com/28303167/165225566-bad11e1d-2d2e-4cb9-8325-cf3636af89a5.mp4
