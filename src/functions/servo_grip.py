from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import json

# --- Configuration (Global Constants) ---

# Vision Parameters
ROI_PCT = 0.20              # Percentage of the top of the frame to ignore
SUCCESS_ROI_PCT = 0.30      # Percentage of the bottom of the frame for the "success" zone
HSV_LOW = np.array([95, 70, 40])   # WIDER/MORE TOLERANT
HSV_HIGH = np.array([130, 255, 255]) # WIDER/MORE TOLERANT
CAL_RATIO = 0.061 * 2 # adjust 
MIN_AREA = 300
ANGLE_TOLERANCE_DEGREES = 2.0 # Angle must be within +/- this value for success
SUCCESS_MASK_PIXEL_RATIO = 0.95 # At least this % of mask pixels must be in the success zone

# --- Gripper Functions ---
def open_gripper(gripper_pwm, cfg):
    """Sets the gripper servo to the open position."""
    print("Opening gripper...")
    gripper_pwm.ChangeDutyCycle(cfg["gripper_duty_open"])
    time.sleep(1)

def close_gripper(gripper_pwm, cfg):
    """Sets the gripper servo to the closed position."""
    print("Closing gripper...")
    gripper_pwm.ChangeDutyCycle(cfg["gripper_duty_close"])
    time.sleep(1)

def main():
    """Main function to run the camera, vision processing, and gripper control loop."""
    
    # --- Gripper and Robot Configuration ---
    # In a real application, this would be loaded from a file like 'config.json'
    # For this example, we'll define it directly in the code.
    cfg = {
        "gripper_pwm": 33,          # The GPIO pin (in BOARD numbering) for the gripper servo
        "gripper_duty_open": 11.5,   # PWM duty cycle for the 'open' position
        "gripper_duty_close": 3.5  # PWM duty cycle for the 'closed' position
    }

    # --- GPIO Setup ---
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["gripper_pwm"], GPIO.OUT)
    gripper_pwm = GPIO.PWM(cfg["gripper_pwm"], 50) # 50Hz PWM frequency
    gripper_pwm.start(cfg["gripper_duty_open"]) # Start with gripper open
    time.sleep(1)

    # --- Camera Setup ---
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)

    # Kernel for morphological operations
    kernel = np.ones((3, 3), np.uint8)
    print("Vision system active. Press 'q' to quit.")

    try:
        # --- Main Loop ---
        while True:
            # 1) Capture a raw image
            raw_frame = picam.capture_array()
            H, W, _ = raw_frame.shape

            # --- ROI Calculations ---
            roi_height = int(H * ROI_PCT)
            success_roi_y_start = H - int(H * SUCCESS_ROI_PCT)

            # Create a separate frame for processing
            processing_frame = raw_frame.copy()
            processing_frame[0:roi_height] = 0

            # 2) Convert to HSV and create a mask
            hsv = cv2.cvtColor(processing_frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 3) Find the largest contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea) if contours else None

            # --- Annotations ---
            display_frame = raw_frame.copy()
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, 0), (W, roi_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

            dash_length = 10
            gap_length = 10
            for i in range(0, W, dash_length + gap_length):
                start_point = (i, success_roi_y_start)
                end_point = (i + dash_length, success_roi_y_start)
                cv2.line(display_frame, start_point, end_point, (0, 255, 255), 2)

            angle_text = "No target"
            # 4) If a contour is found, process it
            if largest_contour is not None and cv2.contourArea(largest_contour) > MIN_AREA:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                image_center = (W // 2, H // 2)
                bb_center = (x + w // 2, y + h // 2)
                cv2.arrowedLine(display_frame, image_center, bb_center, (0, 255, 255), 2)

                center_x = x + w / 2.0
                angle = CAL_RATIO * (center_x - W / 2.0)
                direction = "RIGHT" if angle >= 0 else "LEFT"
                angle_text = f"{direction} {abs(angle):.2f} DEGREES"

                # --- Success Condition Check ---
                is_aligned = abs(angle) < ANGLE_TOLERANCE_DEGREES
                
                total_mask_pixels = cv2.countNonZero(mask)
                if total_mask_pixels > 0:
                    success_roi_mask = mask[success_roi_y_start:H, 0:W]
                    pixels_in_success_zone = cv2.countNonZero(success_roi_mask)
                    pixel_ratio = pixels_in_success_zone / total_mask_pixels
                    is_in_zone = pixel_ratio >= SUCCESS_MASK_PIXEL_RATIO
                else:
                    is_in_zone = False

                # --- Gripper Action Trigger ---
                if is_aligned and is_in_zone:
                    success_text = "IN POSITION"
                    text_size, _ = cv2.getTextSize(success_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
                    text_x = (W - text_size[0]) // 2
                    text_y = (H + text_size[1]) // 2
                    cv2.putText(display_frame, success_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (57, 255, 20), 2)
                    
                    # Update the display one last time before closing
                    display_frame_bgr = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Annotated View", display_frame_bgr)
                    cv2.waitKey(500) # Pause for 0.5 seconds to show the final frame

                    # Close the gripper and break the loop
                    close_gripper(gripper_pwm, cfg)
                    print("Target acquired. Task complete.")
                    time.sleep(2)
                    break 

            # 5) Add text and show results
            cv2.putText(display_frame, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            display_frame_bgr = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("Annotated View", display_frame_bgr)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # --- Cleanup ---
        print("Shutting down...")
        picam.stop()
        cv2.destroyAllWindows()
        gripper_pwm.stop()
        GPIO.cleanup()
        
if __name__ == "__main__":
    main()
