# Raspberry Pi Pick-and-Place Autonomous Robot Build

## The Objective 

My aim is to design and build, from component parts and software modules, a robot capable of autonomous transport--over the unfinished concrete planar inclined surface of a residential swimming pool--of 3D-printed blocks from scattered locations to a designated 2 ft. by 2 ft. drop-off zone, all without colliding with the walls of the swimming pool or undelivered blocks. My motivation for this project is (i) in familiarizing myself with the process of integrating sensors, actuators, and computing boards, (ii) putting into practice some robotics concepts, including navigation, localization, object detection, and drive control, and (iii) establishing a "platform project" that can serve as a physical testing ground as I advance with my University of Maryland (College Park) M.Eng. Robotics Engineering studies, much of which are simulation-based only. 

## The Results to Date
I have integrated the mechanical, vision, drive, and gripper subsystems into a working Raspberry Pi robot that detects, aligns with, and grasps 3D-printed blocks. I've demonstrated square driving, approaching and grasping blocks, and sonar-based relocalization at the drop-off. I'm currently troubleshooting torque-limited stalls and planning to replace the 6 V AA pack with a 7.4 V 2S LiPo, then re-tune the drive control. 

## The Mechanical Sub-System
The robot's body comprises a commercial-off-the-shelf chassis (DFRobot 4WD Baron Mobile Robot Platform), four commercial-off-the-shelf silicone tread orange and clear motor wheels (Adafruit), a commercial-off-the-shelf servo-gripper assembly (ServoCity Servo-Driven Parallel Gripper Kit), a shelf-based mounting assembly I designed and manufactured, and a perception mounting part I designed and manufactured--these latter two I designed in Dassault Systèmes computer-aided design (CAD) program 2024 Solidworks, I prepared for 3-D printing in computer-aided manufacturing (CAM) program Prusa Slicer, and I manufactured on Prusa MK3 and Prusa MK4 fused deposition modeling (FDM) 3-D printers at University of Maryland's Rapid Prototyping Center. I fastened all of the mechanical components with 2 mm and 3 mm machine screws and associated nut, washer, and standoff fastener hardware. The primary challenge with implementing a mechanical subsystem for this robot was in striking a balance between leveraging commercial-off-the-shelf components, which are accessible but come only in a single form, and between leveraging custom-made components, which can be highly tailored to the application at the expense of requiring development time to produce.

The following image collages show the results and the process of my mechanical design and integration efforts to date. 

1. ![Robot_Gallery_1](docs/gallery/Robot_Gallery_1.png)
2. ![Robot_Gallery_3](docs/gallery/Robot_Gallery_3.png)
3. ![Robot_Gallery_4](docs/gallery/Robot_Gallery_4.png)
4. ![Robot_Gallery_2](docs/gallery/Robot_Gallery_2.png)

The mechanical subsystem, the body of the robot, provides a physical structure, provides holding places for each of the robot's sensors, actuators, integrated circuits, and computing boards, and contributes, to the build, through an adjustable shelving system and a front-facing sensor mount that I designed and manufactured progress towards durability, progress towards modularity, and progress towards adaptability. I developed the shelving system so each shelf can be adjusted for height and fastened securely to shelving side panels with M3 machine screws, nuts, and washers. Robot sensors, actuators, and boards are mounted to each shelf with M2 and M3 fastener hardware. Because I 3-D printed each shelf with an infill density of 50%, additional through holes can be drilled without sacrificing structural integrity when component placement needs changing. Additionally, the front-facing mount, holding the Raspberry Pi camera module (Raspberry Pi Camera Module 2) and ultrasonic distance sensor (RCWL-1601), features 2 mm mounting holes for standoff mounts to the camera, a rectangular cutout for the camera's flat-flex ribbon cable, and a rectangular slide-in retention slot for the distance sensor. While the vehicle is in operation, or when the vehicle is shaken from side to side, no component moves from its mounted position. Furthermore, the shelving system doubles as a case that shields the internal components from dust and debris and remains intact when the robot falls over on its side. 

## The Vision Subsystem
The vision subsystem determines (i) if a 3D printed block exists in the robot’s field of vision, (ii) the degree of misalignment between the robot and any identified block, and (iii) whether an identified block is “in position” to be gripped by the robot’s gripper. To achieve these, I implemented a Python-based and OpenCV-based vision processing pipeline based around HSV masking, morphology operations (opening and closing), contour detection, and bounding box drawing. The logic of the module considers the pixel offset of any detected block’s bounding box from the center of the image, converting this offset into a pivot angle command via multiplication by a pixel to angle calibration ratio. 

![Vision Pipeline Table](docs/gallery/Vision_Pipeline_Table.png)

![All Grasp Conditions](docs/gallery/All_Grasp_Conditions.png)

In the following video, I demonstrate the vision subsystem’s ability to detect a 3D printed block, determine the angle of misalignment, and determine whether or not the block is in gripping position.

![Visual Subsystem Demo Screencast](docs/gallery/visual_demo.gif)

[![Visual Subsystem Demo External View](docs/gallery/Visual_Demo_Poster.png)](https://youtu.be/7oZMbLTcFHM)

## The Motor Subsystems

### The Drive Motor Subsystem
The drive motor subsystem assists the robot in achieving any (x, y, psi) pose within its operating environment. I built the drive motor system with four “TT gearbox” DC motors, motor driver (L298N), silicone tread wheels, and emergency shutoff switch. Since the L298N supports two motor channels, and since I wanted to achieve differential drive, I connected both left-side motors to one, and both right-side motors to the other. Regarding sensors, for odometry, I integrated a pair of optical encoders, each mounted to one motor on a different side of the robot, as well as an IMU (BNO055) to track heading. In a Python-based software module, I integrate the sensors and motors, and implement four motion primitive drive functions–two for straight-line motion (forward and reverse) and two for angular pivoting (left pivot and right pivot). Within all four motion functions, I implement feedback control, specifically proportional control. For the straight-line motion functions, the error function derives from IMU heading offset from heading recorded prior to starting motion–for the pivot functions, IMU heading offset from the target heading. All four run within a speed-limited 1 kHz control loop, with the stop condition, for the straight-line motion functions, derived from whether or not encoder ticks equal the required number and for the pivot functions, whether the target heading is within tolerance of the current measurement. 

With the following demonstration, I show the robot’s capability to drive in a square: 

[![Square Demo](docs/gallery/Square_Demo_Poster.png)](https://youtu.be/4IkJXeA65EI)


Here, I have tabulated the primary components, electrical connections, and key specifications for the components of the drive subsystem, with the first table detailing the motor and motor driver and the second table detailing the sensors. 

![Drive Wiring Motors](docs/gallery/Drive_Wiring.png)

![Drive Wiring Sensors](docs/gallery/Drive_Wiring_Sensors.png)

### The Gripper Motor Subsystem
The gripper motor subsystem enables the robot to pick-and-place 3D printed blocks. After the robot identifies a 3D printed block and maneuvers until the block is in position, the gripper activates, grasping the object. While in transit to the destination block drop-off zone, the gripper keeps a firm hold on the block, keeping the block from dropping. When the robot reaches the drop-off zone, the gripper activates again, dropping the block in place. The subsystem comprises a servo-gripper kit (Proton Servo), which I mounted to the front of the robot chassis. The gripper’s servo motor responds to pulse-width modulation (PWM). I implement a Python-based software module to provide functions for opening and closing the gripper. 

In the following video, I demonstrate the robot gripping a 3D printed block, holding it for 5 seconds, then dropping the 3D printed block. 

[![Gripper Demo](docs/gallery/Gripper_Demo_Poster.png)](https://youtu.be/0rDtxxSHDeY)

## The Visual-Motor Integration
The vision and motor subsystems work together. The vision subsystem provides the means for determining the relative position of blocks from the camera. The motor subsystem provides the means for approaching the 3D printed block, then grasping the 3D printed block when the conditions for grasping are met. 

In the following demonstration, I show the robot tracking the 3D printed block, grasping it once in position, reversing slightly, then dropping the block. 

![Grasp Demo](docs/gallery/Grasp.gif)

## The Navigation Subsystem
The navigation subsystem provides a means for tracking the robot’s pose throughout the mission. I’ve implemented a software module for navigation that incorporates both interoceptive sensor data (encoders and IMU) and exteroceptive sensor data (sonar). The reason I incorporate both sensor types is to address the sensor-drift challenge of estimated pose from interoceptive sensors alone. The robot tracks its pose based on interoceptive sensor measurements after each motion. When the pose estimate falls within a known corner of the map, the drop-off zone, the robot performs a relocalization routine. The robot spins 360 degrees, collecting sonar measurements. My software processes the sonar trace by smoothing it to reduce noise, identifying plateau regions corresponding to walls, and extracting the minimum distance reading from each plateau to estimate the perpendicular distance to each wall.

The following video is a demonstration of the robot executing the relocalization routine at the drop-off site. 

[![Relocalize Demo](docs/gallery/Relocalize_Demo_Poster.png)](https://youtu.be/HixW7EjjSHo)

The following figure illustrates the sonar trace, with walls showing up as plateaus. 
![Sonar Trace](docs/gallery/Sonar_Trace.png)

## Follow Up: Sept. 2, 2025: Troubleshooting Unreliable Driving
This project is a work in progress. I’m currently troubleshooting unreliable pivot maneuvers. Wheel slippage improved after switching from plastic to silicone tires, but not completely. I plan to try solving this by coating the tires with hot glue–in a tread-like pattern. A second issue is insufficient motor torque. The added weight from the 3D-printed mounts and casing, combined with powering four motors from a 6 V AA battery pack through a single L298N motor driver (which drops ~ 2 V internally), leaves too little voltage–and therefore current–at the motor terminals. As a result, the robot frequently stalls during pivots, especially on inclines, even at full PWM. To address this, I plan to replace my 6 V AA battery pack with a 7.4 V 5000 mAh 50 C high discharge LiPo battery. 

![Next Steps](docs/gallery/Next_Steps.JPG)

## Follow Up: Sept 8, 2025: Installation of the LiPo Battery
I have installed the 7.4 V 5000 mAh 50 C high discharge LiPo battery. I've included a picture showing the main wiring. I have wired the battery to the L298N supply voltage terminals. I use a 7.5 A inline fuse. Also, because the 12 AWG battery wires are too thick for the motor driver, I use 16 AWG wire, which I connect to the larger gauge wire with a lap-joint solder connection. 

While preparing for this installation, I learned that before I begin testing driving performance on the swimming pool floor, I should probably swap my L298N with a motor driver rated for higher currents. The reason for this is that the stall current for each of my motors is probably ~ 2 A, but the maximum supply current, per output channel, as per the L298 STMicroelectronics datasheet is 2 A. I've purchased a BTS7960 (43 A max. supply current) to replace my L298N, and will proceed with the installation upon its arrival in the mail. 

I've conducted bench no-load motor testing (robot suspended above ground) with the new battery wired to my L298N, and the motors roar at 80 PWM, expected because of the higher voltage level. To avoid a thermal shutdown scenario, I will wait to install the new motor driver before I have the robot drive on the swimming pool surface. 

![LiPo Wiring](docs/gallery/LiPo_Wiring.png)

## Follow Up: Sept. 17, 2025: Modifications to the Motor Control Circuit

I have decided to upgrade the entire motor control circuit, not just the battery. The buck converter serves the purpose of stepping down the battery voltage from 7.4 V to 6 V for the TT gear motors. The new motor driver, the DBH-12 is rated for a much higher output current per channel vs. the L298N, 20 A vs 2 A, so stall-induced overheating will be a nonissue. Because of the LiPo’s high discharge rate and low internal resistance, I have added an inline 7.5 A blade fuse to protect against short circuits, though I may decide to replace this with a 10 A fuse to prevent tripping during motor stalls. 

![New Motor Control Components](docs/gallery/New_Motor_Circuit_Components.JPG)
![Motor Control Circuit Upgrade](docs/gallery/Motor_Control_Upgrade_1.png)

## Follow Up: Sept. 17, 2025: Integrating the New Motor Control Hardware 

Today was a fun day. I successfully installed the DBH-12 motor driver and 300 W buck converter. The following is video from bench testing. I managed to blow the LiPo inline fuse twice upon accidentally touching a screwdriver between the buck’s voltage output screw terminal and its grounded heatsink. It’s a good thing I have more blade fuses on hand. The motors seem significantly more powerful than they were with the old motor driver. I’m measuring a ~0.5 V sag on no-load motor drive at 100% duty cycle. When I press down my fingers to induce a stall, that sag grows to ~1 V total. I’m beginning to think about reasons why this might be happening and how to minimize this sag. When my replacement DMM fuses arrive in the mail–I previously blew my 10 A DMM fuse–I’ll use my DMM to measure the current across the buck’s output terminals to check if my constant current buck setting is a contributing factor. 

[![Bench Test New Motor Hardware](docs/gallery/New_Motor_HW_Test.png)](https://youtu.be/4IkJXeA65EI)