# Raspberry Pi Pick-and-Place Autonomous Robot Build

## Objective 

My aim is to design and build, from component parts and software modules, a robot capable of autonomous transport--over the unfinished concrete planer inclined surface of a residential swimming pool--of 3D-printed blocks from scattered locations to a designated 2 ft. by 2 ft. drop-off zone, all without colliding with the walls of the swimming pool or undelivered blocks. My motivation for this project is (i) in familiarizing myself with the process of integrating sensors, actuators, and computing boards, (ii) putting into practice some robotics concepts, including navigation, localization, object detection, and drive control, and (iii) establishing a "platform project" that can serve as a physical testing ground as I advance with my University of Maryland (College Park) M.Eng. Robotics Engineering studies, much of which is simulation-based only. 

## Mechanical Sub-System
The robot's body comprises a commercial-off-the-shelf chassis (DFRobot 4WD Baron Mobile Robot Platform), four commercial-off-the-shelf silicone tread orange and clear motor wheels (Adafruit), a commercial-off-the-shelf servo-gripper assembly (ServoCity Servo-Driven Parallel Gripper Kit), a shelf-based mounting assembly I designed and manufactured, and a perception mounting part I designed and manufactured--these latter two I designed in Dassault Systèmes computer aided design (CAD) program 2024 Solidworks, I prepared for 3-D printing in computer aided manufacturing (CAM) program Prusa Slicer, and I manufactured on Prusa MK3 and Prusa MK4 fusion deposition modeling (FDM) 3-D printers at University of Maryland's Rapid Prototyping Center. I fastened all of the mechanical components with 2 mm and 3 mm machine screws and associated nut, washer, and standoff fastener hardware. The primary challenge with implementing a mechanical subsystem for this robot was in striking a balance between leveraging commercial-off-the-shelf components, which are accessible but come only in a single form, and between leveraging custom-made components, which can be highly tailored to the application at the expense of requiring development time to produce.

The following image collages show the results and the process of my mechanical design and integration efforts to date. 

1. ![Robot_Gallery_1](docs/gallery/Robot_Gallery_1.png)
2. ![Robot_Gallery_3](docs/gallery/Robot_Gallery_3.png)
3. ![Robot_Gallery_4](docs/gallery/Robot_Gallery_4.png)
4. ![Robot_Gallery_2](docs/gallery/Robot_Gallery_2.png)

The mechanical subsystem, the body of the robot, provides a physical structure, provides holding places for each of the robot's sensors, actuators, integrated circuits, and computing boards, and contributes, to the build, through an adjustable shelving system and a front-facing sensor mount that I designed and manufactured progress towards durability, progress towards modularity, and progress towards adaptability. I developed the shelving system so each shelf can be adjusted for height and fastened securely to shelving side panels with M3 machine screws, nuts, and washers. Robot sensors, actuators, and boards are mounted to each shelf with M2 and M3 fastener hardware. Because I 3-D printed each shelf with an infill density of 50%, additional through holes can be drilled without sacrificing structural integrity when component placement needs changing. Additionally, the front-facing mount, holding the Raspberry Pi camera module (Raspberry Pi Camera Module 2) and ultrasonic distance sensor (RCWL-1601), features 2 mm mounting holes for standoff mounts to the camera, a rectangular cutout for the camera's flat-flex ribbon cable, and a rectangular slide-in retention slot for the distance sensor. While the vehicle is in operation, or when the vehicle is shaken from side to side, no component moves from its mounted position. Furthermore, the shelving system doubles as a case that shields the internal components from dust and debris and remains intact when the robot falls over on its side. 

## Vision Subsystem
The primary challenge that the robot's vision subsystem addresses is the challenge of needing to know first, whether a 3D-printed block exists in the robot's field of view, and second, the degree of yaw-angle misalignment between the robot and the 3D-printed block. A secondary challenge is that the robot will not be able to process camera image frames in real time due to the fact that the computational processing capability of the Raspberry Pi 4 is insufficient for doing real-time image processing for object detection. The solution I designed is a straightforward color-masking pipeline for 3D-printed block detection comprising seven steps. So long as this pipeline is executed in lower-than-real-time frame rates, the Raspberry Pi should be able to handle the computational demand that the pipeline introduces.  

The seven steps of the image processing pipeline are as follows: 

**1. Raw Image Capture.** The open CV function is named. The inputs are named and the outputs are named. An example photo is provided. `Picam2().capture_array()`

**2. Region of Interest Crop.** `bgr[:int(Percent * H)] = 0`

**2.5 BGR to HSV Conversion** `cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)`

**3. Mask Image** `cv2.inRange(hsv, thresh_low, thresh_high)`

**4. Detect Contours** `cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)`

**5. Select Largest Contour.** `max(countours, key=cv2.contourArea)`

**6. Bounding Box Draw.** `x, y, w, h = cv2.boundingRect(largest_contour)`

**7. Pivot Angle Estimation** `pivot_angle = calibration_ratio * (x_bb - x_image)` 


## Drive Sub-System




Explain that the drive sub-system is comprised of four DC motors driven by an H-bridge motor driver that supports two channels, one for the left-side motors, and the other for the right-side motors. Explain that I developed four motion primitive functions to drive the robot: pivot_left, pivot_right, forward, reverse. Explain that the forward and reverse functions employ a while loop that iterates on distance traveled as measured by encoder ticks and also a P-controller with feedforward having an error function that is the error in initial heading. Explain that the pivot functions employ a simple while loop over error in current heading vs desired heading. Explain that the sensors are an IMU and optical encoders. 

**Show a video of the robot driving in a square.** 

Algorithm
Set distance
Compute ticks
Read IMU heading
While Average Ticks < Required Average
- Compute error in heading
- Change Duty Cycle
- Sleep 100 ms
Stop Motors 

## Navigation Sub-System
Explain that the navigation subsystem is basically a method that updates the state of the robot after the completion of each robot. Explain that this state is an estimate and that I understand that this will grow in inaccuracy during the mission, necessitating a relocalization technique. Explain that the relocalization technique involves spinning in a circle and computing the distance to 2 walls, thereby inferring position on a pre-loaded map (obstacle free, 10 ft by 10 ft rectangle). 

**Insert a FPV video of spinning in a circle and then showing the distance measurements with local minima identified. (use numpy or something). Show the MATPLOTLIB time plot of distance readoings with the minima identified.**


Algorithm: 
1. For each action update estimate. 
2. When belief is within drop-off zone, relocalize. 
3. Replace pose estimate with result of relocalization. 

## Gripper Sub-System
Explain that the gripper sub-system is comprised of a COTS gripper sub-assembly and servo motor, plus a software interface I designed to open and close the gripper.  

**Insert a FPV video of the robot approaching a block and gripping it successfully.** 

Robot
 members: state
 functions: pivot functions, camera functions, etc.

## Earlier Tests

