# DeltaController
Run on an Arduino Nano, this controller uses inverse kinematics to control a Delta robot.
The controller takes in serial commands and mvoes the robot accordingly.

## Usage
After defining link lengths laid out in the Kinematics.h file and uploading the code to the dev board, 
one can send a linear move command (LM) followed by the x, y and z coords seperated with spaces.

example Command sent over serial: 
```
LM -10.0 25.0 250.0
```
This will, if in bounds, move the robot to an x position of -10mm, a y position of 25mm and a z position of 250mm

More commands are being added. 
