import RPi.GPIO as GPIO
import time
import json
import numpy as np
from enum import Enum
import serial
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import math

# ===== Parameters for Grasping Demonstration =====
# Vision Parameters
ROI_PCT  = 0.20                            # Percentage of top of frame to ignore
HSV_LOW  = np.array([95, 70, 40])          # Blue HSV mask low 
HSV_HIGH = np.array([130, 255, 255])       # Blue HSV mask high 
BASE_CAL_RATIO = 0.061                     # Instructor provided pixle : angle ratio at 640 px x 480 px
SCALE_FACTOR   = 2                         # Demo uses 320 px x 240 px, so scale the ratio
CAL_RATIO = SCALE_FACTOR * BASE_CAL_RATIO  # Scale the provided ratio to the Picam demo resolution
MIN_AREA             = 300                 # Minimum required contour for object detection
ERROR_THRESHOLD_DEG  = 3.0                 # Maximum absolute deviation requirement for being "in position"
SUCCESS_ROI_PCT      = 0.30                # Percentage of the bottom frame for "success" zone
SUCCESS_MASK_PIXEL_RATIO = 0.95            # Percent of masked pixels required to be in success zone for being "in position"

# Motion Control Parameters
PIVOT_DUTY_CYCLE       = 95    # Base duty cycle for pivot controller
ALIGN_STEP_DEG         = 5     # Pivot alignment step (degrees), right(+), left (-)
SEARCH_STEP_DEG        = 10    # Pivot spin search increment
APPROACH_STEP_CM       = 5.0   # Forward step distance
MAX_APPROACH_STEPS     = 60    # Stop safety distance

# Grasp Parameters
BACKUP_AFTER_GRASP_CM   = 8.0   # How far to reverse after grasping

# ===== Functions Necessary for Robot Driving ===== 
class Motion(Enum):
    FORWARD = 1
    REVERSE = 2
    PIVOT_LEFT = 3
    PIVOT_RIGHT = 4
    STOP = 5

def set_logic(mode: Motion, cfg):
    if mode == Motion.FORWARD:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.HIGH)
    elif mode == Motion.REVERSE:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.LOW)
    elif mode == Motion.PIVOT_LEFT:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.HIGH)
    elif mode == Motion.PIVOT_RIGHT:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.LOW)

def read_heading(ser):
    ser.write(b'R')
    line = ser.readline()
    if not line:
        return None
    try:
        return int(line.decode(errors='ignore').strip())
    except ValueError:
        return None

def wrap_angle(raw_angle):
    return ((raw_angle + 180) % 360) - 180

def pivot(mode, pivot_angle_deg, cfg, left_motor_pwm, right_motor_pwm, ser):
    set_logic(mode, cfg)

    base_duty = PIVOT_DUTY_CYCLE
    kp = 0.0  # Because motors currently supply insufficient torque, always apply base duty for pivots
              # and do not use proportional control until fixed motor issues are fixed.

    start_heading = read_heading(ser)
    if start_heading is None:
        start_heading = 0
    if mode == Motion.PIVOT_LEFT:
        target_heading = wrap_angle(start_heading - pivot_angle_deg)
    elif mode == Motion.PIVOT_RIGHT:
        target_heading = wrap_angle(start_heading + pivot_angle_deg)
    else:
        target_heading = start_heading

    error = wrap_angle(target_heading - (read_heading(ser) or start_heading))
    # spin until we hit the target (approx); with kp=0, we just run base duty
    while abs(error) > 2:
        left_motor_pwm.ChangeDutyCycle(base_duty)
        right_motor_pwm.ChangeDutyCycle(base_duty)
        error = wrap_angle(target_heading - (read_heading(ser) or start_heading))
        time.sleep(0.005)

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)

def drive_line_imu(mode, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser):
    set_logic(mode, cfg)

    ticks_per_rev = cfg["ticks_per_rev"]
    wheel_diameter_cm = cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm = np.pi * wheel_diameter_cm
    target_ticks = target_distance_cm / wheel_circumference_cm * ticks_per_rev

    prev_state_left  = GPIO.input(cfg["left_encoder_pin"])
    prev_state_right = GPIO.input(cfg["right_encoder_pin"])
    count_left, count_right = 0, 0

    base_heading = read_heading(ser) or 0
    base_duty = 50
    kp = 10

    while (count_left + count_right) / 2 < target_ticks:
        cur_left  = GPIO.input(cfg["left_encoder_pin"])
        cur_right = GPIO.input(cfg["right_encoder_pin"])

        if cur_left != prev_state_left:
            count_left += 1
            prev_state_left = cur_left
        if cur_right != prev_state_right:
            count_right += 1
            prev_state_right = cur_right

        current_heading = read_heading(ser) or base_heading
        heading_error = wrap_angle(base_heading - current_heading)

        if heading_error < 0:
            adj = kp * abs(heading_error)
            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adj, 0, 100))
            else:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adj, 0, 100))
        elif heading_error > 0:
            adj = kp * abs(heading_error)
            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adj, 0, 100))
            else:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adj, 0, 100))
        else:
            left_motor_pwm.ChangeDutyCycle(base_duty)
            right_motor_pwm.ChangeDutyCycle(base_duty)

        time.sleep(0.001)

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)

# ===== Functions Necessary for Gripper Control =====
def open_gripper(gripper_pwm, cfg):
    duty = cfg.get("gripper_duty_open", 11.5)
    gripper_pwm.ChangeDutyCycle(duty)
    time.sleep(1.0)

def close_gripper(gripper_pwm, cfg):
    duty = cfg.get("gripper_duty_close", 3.5)
    gripper_pwm.ChangeDutyCycle(duty)
    time.sleep(1.0)

# ===== Functions Necessary for Visual Processing =====
def get_angle_and_success(mask, W, H):
    
    # 7. Contour Detection: Computes the boundaries of groupings of activated mask pixels. 
    #    Reason: Contours are the outline of detected objects. Finding contours means 
    #            determining coordinate information regarding likely 3D printed blocks
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 8. Largest Contour Filtering: Selects only the largest of all of these masked pixel groupings.
    #    Reason: The largest contour is likely to be the closest block to the robot 
    largest = max(contours, key=cv2.contourArea) if contours else None
    
    
    angle = None

    # 9. Pivot Angle Estimation: Converts the pixel offset into a pivot angle estimate. 
    #    Reason: The estimated pivot angle provides a direct control input for correcting robot misalignment with the block.
    if largest is not None and cv2.contourArea(largest) > MIN_AREA:
        x, y, w, h = cv2.boundingRect(largest)
        center_x = x + w / 2.0
        angle = CAL_RATIO * (center_x - W / 2.0) 

    # 10. Proximity Calculation: Almost all masked pixels lie within the designated “success zone” at the bottom of the image.
    #     Reason: Confirms the block is close enough to the robot’s gripper for a successful grasp. 
    success_y_start = H - int(H * SUCCESS_ROI_PCT)
    total_mask_pixels = cv2.countNonZero(mask)
    if total_mask_pixels > 0:
        success_roi_mask = mask[success_y_start:H, 0:W]
        pixels_in_success = cv2.countNonZero(success_roi_mask)
        ratio = pixels_in_success / total_mask_pixels
        in_zone = ratio >= SUCCESS_MASK_PIXEL_RATIO
    else:
        in_zone = False

    return angle, in_zone

# ===== Grasp Demonstration =====
def main():
    
    # Load the robot configuration parameters
    with open("config.json") as f:
        cfg = json.load(f)

    
    # Set GPIO and instantiate PWM for all drive motors
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    GPIO.setup(cfg["right_encoder_pin"], GPIO.IN)
    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 400)
    left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(cfg["right_motor_pwm"], 400)
    right_motor_pwm.start(0)

    # Set GPIO and PWM for gripper
    gripper_pin = cfg.get("gripper_pwm", 33)
    GPIO.setup(gripper_pin, GPIO.OUT)
    gripper_pwm = GPIO.PWM(gripper_pin, 50)
    gripper_pwm.start(cfg.get("gripper_duty_open", 11.5))  # start open
    time.sleep(0.5)

    # Set up the IMU
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    time.sleep(2.0)
    ser.reset_input_buffer()

    # Set up the Raspberry Pi camera
    picam = Picamera2()
    cam_cfg = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(cam_cfg)
    picam.start()
    time.sleep(0.2)

    # Create a kernel for morphology operations
    kernel = np.ones((3, 3), np.uint8)
    
    
    print("Grasp Demo Started. Press Ctrl+C to STOP.")

    try:
        
        # Track 
        steps_taken = 0
        while True:
            
            
            # 1. Raw Image Capture: Captures a raw RGB image with the Pi camera. 
            #    Reason: The image is the basic unit of analysis for this perception pipeline   
            frame = picam.capture_array()
            H, W, _ = frame.shape

            # 2. ROI Crop: Crops out the top portion of the image. 
            #    Reason: The top portion of the image is NOT ground plane and ONLY increases 
            #            the odds of false object detection. 
            proc = frame.copy()
            roi_height = int(H * ROI_PCT)
            proc[0:roi_height] = 0

    
            # 3. HSV Conversion: Converts the cropped image into HSV colorspace. 
            #    Reason: HSV provides more robust means for object detection based on color            
            hsv = cv2.cvtColor(proc, cv2.COLOR_RGB2HSV)
            
            # 4. Blue Color Mask: Creates a binary mask where activated pixels are HSV blue.
            #    Reason: The 3D printed blocks are all blue. This step will identify any present in the image.         
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            
            # 5. Morphological Operning: Reduces “salt noise” in the HSV mask image.
            #    Reason: Masking typically produces some degree of salt noise.  
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 6. Morphological Closing: Patches small holes in the HSV mask image. 
            #    Reason: Masking often activates most but not all pixels along continuous domains. 
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Verify all required conditions for grasping
            angle, in_zone = get_angle_and_success(mask, W, H)
            
            # When the robot fails to identify a target, it will pivot to the RIGHT by a step increment, then SKIP
            # to the next iteration of this main loop, to look again. 
            if angle is None:  
                print("NO SEE TARGET. SPINNING SOME, AND LOOKING AGAIN")
                pivot(Motion.PIVOT_RIGHT, SEARCH_STEP_DEG, cfg, left_motor_pwm, right_motor_pwm, ser)
                time.sleep(0.05)
                continue

            # --- Verify All Conditions Required for Grasping ---
            
            # Necessary Condition 1: Alignment: The estimated pivot angle is within a small tolerance of 0.0°.
            # Reason: Almost all masked pixels lie within the designated “success zone” at the bottom of the image.
            # WHEN this condition is NOT met, the robot will pivot either left or right by `ALIGN_STEP_DEG` to improve alignment.
            if abs(angle) > ERROR_THRESHOLD_DEG:    
                if angle > 0:
                    pivot(Motion.PIVOT_RIGHT, min(ALIGN_STEP_DEG, abs(angle)), cfg, left_motor_pwm, right_motor_pwm, ser)
                else:
                    pivot(Motion.PIVOT_LEFT,  min(ALIGN_STEP_DEG, abs(angle)), cfg, left_motor_pwm, right_motor_pwm, ser)
                time.sleep(0.02)
                continue  

            # Necessary Condition 2: Proximity: Almost all masked pixels lie within the designated “success zone” at the bottom of 
            # the image.
            # Reason: Confirms the block is close enough to the robot’s gripper for a successful grasp. 
            if not in_zone:
                if steps_taken >= MAX_APPROACH_STEPS: # Safety stop
                    break
                drive_line_imu(Motion.FORWARD, APPROACH_STEP_CM, cfg, left_motor_pwm, right_motor_pwm, ser)
                steps_taken += 1
                time.sleep(0.02)
                continue  
            
            # When all grasp requirements are met, then grasp, back up, then drop. 
            close_gripper(gripper_pwm, cfg)        
            drive_line_imu(Motion.REVERSE, BACKUP_AFTER_GRASP_CM, cfg, left_motor_pwm, right_motor_pwm, ser)
            open_gripper(gripper_pwm, cfg)
            
            time.sleep(1.0)
            break

    except KeyboardInterrupt:
        pass 
    finally:
        print("Cleaning up...")
        try:
            picam.stop()
        except Exception:
            pass
        left_motor_pwm.stop()
        right_motor_pwm.stop()
        gripper_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
