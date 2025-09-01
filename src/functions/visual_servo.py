import RPi.GPIO as GPIO
import time
import json
import numpy as np
from enum import Enum
import serial
import cv2
from picamera2 import Picamera2
from libcamera import Transform

# --- Vision Configuration ---
ROI_PCT = 0.20
HSV_LOW = np.array([95, 70, 40])
HSV_HIGH = np.array([130, 255, 255])
CAL_RATIO = 0.061 * 2
MIN_AREA = 300
ERROR_THRESHOLD_DEG = 3.0 # The tolerance for being "aligned" in degrees

# --- Motor Configuration ---
PIVOT_DUTY_CYCLE = 95# Speed of the pivot maneuver (0-100)

class Motion(Enum):
    """Enum to represent the different types of motion."""
    FORWARD = 1
    REVERSE = 2
    PIVOT_LEFT = 3
    PIVOT_RIGHT = 4
    STOP = 5

def set_logic(mode: Motion, cfg):
    """Sets the motor driver logic pins for the desired motion."""
    if mode == Motion.PIVOT_LEFT:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.HIGH)
    elif mode == Motion.PIVOT_RIGHT:
        GPIO.output(cfg["left_motor_logic_1"], GPIO.LOW)
        GPIO.output(cfg["left_motor_logic_2"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_1"], GPIO.HIGH)
        GPIO.output(cfg["right_motor_logic_2"], GPIO.LOW)
    # Add other motion logic if needed in the future
    # elif mode == Motion.FORWARD: ...

def main():
    """Main function to run the camera, vision processing, and motor control loop."""
    # --- Hardware Setup ---
    cfg = None
    with open("config.json") as f:
        cfg = json.load(f)

    # GPIO Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_2"], GPIO.OUT)

    # PWM Setup
    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 400)
    left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(cfg["right_motor_pwm"], 400)
    right_motor_pwm.start(0)

    # Camera Setup
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)

    # Vision Processing Kernel
    kernel = np.ones((3, 3), np.uint8)
    print("Starting headless visual servo loop. Press Ctrl+C to quit.")

    try:
        # --- Main Visual Servo Loop ---
        while True:
            # 1) Capture and process image to find the target
            raw_frame = picam.capture_array()
            H, W, _ = raw_frame.shape

            # Black out the Region of Interest (ROI) for processing
            processing_frame = raw_frame.copy()
            roi_height = int(H * ROI_PCT)
            processing_frame[0:roi_height] = 0

            # Convert to HSV and create a mask to isolate the target color
            hsv = cv2.cvtColor(processing_frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find the largest contour in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea) if contours else None

            # Initialize angle error
            angle = None
            
            # 2) Calculate angle error if a target is found
            if largest_contour is not None and cv2.contourArea(largest_contour) > MIN_AREA:
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w / 2.0
                angle = CAL_RATIO * (center_x - W / 2.0)

            # 3) Control motors based on the calculated angle error
            if angle is None:
                # No target found, stop motors
                left_motor_pwm.ChangeDutyCycle(0)
                right_motor_pwm.ChangeDutyCycle(0)
                print("NO TARGET")

            elif abs(angle) <= ERROR_THRESHOLD_DEG:
                # Target is centered, stop motors
                left_motor_pwm.ChangeDutyCycle(0)
                right_motor_pwm.ChangeDutyCycle(0)
                print(f"ALIGNED: {angle:.2f} DEG")

            elif angle > ERROR_THRESHOLD_DEG:
                # Target is to the right, pivot right
                set_logic(Motion.PIVOT_RIGHT, cfg)
                left_motor_pwm.ChangeDutyCycle(PIVOT_DUTY_CYCLE)
                right_motor_pwm.ChangeDutyCycle(PIVOT_DUTY_CYCLE)
                print(f"PIVOT RIGHT. Error: {abs(angle):.2f} DEG")

            elif angle < -ERROR_THRESHOLD_DEG:
                # Target is to the left, pivot left
                set_logic(Motion.PIVOT_LEFT, cfg)
                left_motor_pwm.ChangeDutyCycle(PIVOT_DUTY_CYCLE)
                right_motor_pwm.ChangeDutyCycle(PIVOT_DUTY_CYCLE)
                print(f"PIVOT LEFT. Error: {abs(angle):.2f} DEG")
            
            # Add a small delay to prevent excessive CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping loop.")
    finally:
        # --- Cleanup ---
        print("Shutting down and cleaning up GPIO...")
        picam.stop()
        left_motor_pwm.stop()
        right_motor_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
