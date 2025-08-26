# Raspberry Pi Pick-and-Place Autonomous Robot Build

## Objective 

## Results

## Mechanical Sub-System
Explain that the mechanical sub-system is comprised of commercial-off-the-shelf chasis unit, plus an FDM shelving unit and FDM perception sensor mount I designed and build with Solidworks and a Prusa 3D FDM printer in UMD Rapid Prototyping Center.

**Insert a picture of the robot shelving assembly with the main chasis.** 
**Insert a picture of the sensor mount with the main chasis.**

## Vision Sub-System
Explain that the vision sub-system is comprised of a Raspberry Pi camera module and software I developed for block detection. The software utilizes HSV masking logic to identify blocks of a certain color. It also utilizes a calibration constant (pixel to angle) to estimate a pivot angle to the center of each object identified in an image. The output of this module is thus used as input to the drive system, which is responsible for driving to the identified block.   

**Insert a picture of one processed camera image, showing bounding box drawn around the target block with an arrow pointing to its BB center and a pivot direction and pivot angle on the processed image as text.**

## Drive Sub-System
Explain that the drive sub-system is comprised of four DC motors driven by an H-bridge motor driver that supports two channels, one for the left-side motors, and the other for the right-side motors. Explain that I developed four motion primitive functions to drive the robot: pivot_left, pivot_right, forward, reverse. Explain that the forward and reverse functions employ a while loop that iterates on distance traveled as measured by encoder ticks and also a P-controller with feedforward having an error function that is the error in initial heading. Explain that the pivot functions employ a simple while loop over error in current heading vs desired heading. Explain that the sensors are an IMU and optical encoders. 

**Show a video of the robot driving in a square.** 

## Navigation Sub-System
Explain that the navigation subsystem is basically a method that updates the state of the robot after the completion of each robot. Explain that this state is an estimate and that I understand that this will grow in inaccuracy during the mission, necessitating a relocalization technique. Explain that the relocalization technique involves spinning in a circle and computing the distance to 2 walls, thereby inferring position on a pre-loaded map (obstacle free, 10 ft by 10 ft rectangle). 

**Insert a FPV video of spinning in a circle and then showing the distance measurements with local minima identified. (use numpy or something). Show the MATPLOTLIB time plot of distance readoings with the minima identified.**

## Gripper Sub-System
Explain that the gripper sub-system is comprised of a COTS gripper sub-assembly and servo motor, plus a software interface I designed to open and close the gripper.  

**Insert a FPV video of the robot approaching a block and gripping it successfully.** 

Robot
 members: state
 functions: pivot functions, camera functions, etc.
