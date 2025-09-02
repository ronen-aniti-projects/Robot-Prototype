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

# ===================== Vision & Control Parameters =====================
# Vision thresholds (same spirit as your demo)
ROI_PCT                = 0.20
HSV_LOW                = np.array([95, 70, 40])
HSV_HIGH               = np.array([130, 255, 255])
CAL_RATIO              = 0.061 * 2     # pixel-to-deg mapping (you already tuned this)
MIN_AREA               = 300
ERROR_THRESHOLD_DEG    = 3.0           # heading tolerance to be considered "aligned"
SUCCESS_ROI_PCT        = 0.30          # bottom band for "close enough"
SUCCESS_MASK_PIXEL_RATIO = 0.95        # mask density in success band

# Motion control
PIVOT_DUTY_CYCLE       = 95            # raw pivot speed (0-100) for direct pivot PWM
ALIGN_STEP_DEG         = 5             # pivot increments for alignment (right is positive)
SEARCH_STEP_DEG        = 10            # pivot increments during search
APPROACH_STEP_CM       = 5.0           # forward chunk distance (re-check vision each step)
MAX_APPROACH_STEPS     = 60            # safety stop (~3 m) to avoid runaway

# Post-grasp behavior
BACKUP_AFTER_GRASP_CM   = 8.0   # how far to reverse before dropping
DROP_SETTLE_S           = 0.6   # brief pause to let robot stop rocking
POST_DROP_FORWARD_CM    = 2.0   # optional: move forward a touch after drop (0 to disable)


# ===================== Robot Motion (barebones) =====================
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
    """
    Minimal pivot: we use IMU heading readings to stop near the target.
    Uses commanded angle; if your IMU steps are coarse, small increments are more reliable.
    """
    set_logic(mode, cfg)

    base_duty = PIVOT_DUTY_CYCLE
    kp = 0.0  # simple open-loop duty; IMU used only to know when to stop

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
    """
    Minimal straight-line with IMU trim; called in small chunks (e.g., 5 cm).
    """
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

# ===================== Gripper =====================
def open_gripper(gripper_pwm, cfg):
    duty = cfg.get("gripper_duty_open", 11.5)
    gripper_pwm.ChangeDutyCycle(duty)
    time.sleep(1.0)

def close_gripper(gripper_pwm, cfg):
    duty = cfg.get("gripper_duty_close", 3.5)
    gripper_pwm.ChangeDutyCycle(duty)
    time.sleep(1.0)

# ===================== Visual Helpers =====================
def get_angle_and_success(mask, W, H):
    """
    Returns:
      angle_deg (float or None): +right / -left (your convention),
      is_in_success_zone (bool)
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(contours, key=cv2.contourArea) if contours else None
    angle = None

    if largest is not None and cv2.contourArea(largest) > MIN_AREA:
        x, y, w, h = cv2.boundingRect(largest)
        center_x = x + w / 2.0
        angle = CAL_RATIO * (center_x - W / 2.0)  # +right, -left

    # success ROI test
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

# ===================== Main Integrated Demo =====================
def main():
    # ---- Load config ----
    with open("config.json") as f:
        cfg = json.load(f)

    # ---- GPIO Setup ----
    GPIO.setmode(GPIO.BOARD)
    # Motor pins
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    GPIO.setup(cfg["right_encoder_pin"], GPIO.IN)

    # PWM Setup
    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 400); left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(cfg["right_motor_pwm"], 400); right_motor_pwm.start(0)

    # Gripper pins (fallback defaults if not in cfg)
    gripper_pin = cfg.get("gripper_pwm", 33)
    GPIO.setup(gripper_pin, GPIO.OUT)
    gripper_pwm = GPIO.PWM(gripper_pin, 50)
    gripper_pwm.start(cfg.get("gripper_duty_open", 11.5))  # start open
    time.sleep(0.5)

    # Serial IMU
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    time.sleep(2.0)
    ser.reset_input_buffer()

    # Camera
    picam = Picamera2()
    cam_cfg = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(cam_cfg)
    picam.start()
    time.sleep(0.2)

    kernel = np.ones((3, 3), np.uint8)
    print("Visual servo + drive demo started. Ctrl+C to stop.")

    try:
        # ---- State machine loop ----
        steps_taken = 0
        while True:
            frame = picam.capture_array()
            H, W, _ = frame.shape

            # Ignore top ROI
            proc = frame.copy()
            roi_height = int(H * ROI_PCT)
            proc[0:roi_height] = 0

            hsv = cv2.cvtColor(proc, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            angle, in_zone = get_angle_and_success(mask, W, H)

            if angle is None:
                # --- SEARCH: no target; pivot in chunks to scan ---
                print("NO TARGET -> SEARCH PIVOT")
                pivot(Motion.PIVOT_RIGHT, SEARCH_STEP_DEG, cfg, left_motor_pwm, right_motor_pwm, ser)
                time.sleep(0.05)
                continue

            # --- ALIGN: pivot to reduce heading error ---
            if abs(angle) > ERROR_THRESHOLD_DEG:
                # Right is +, Left is - (your convention).
                if angle > 0:
                    print(f"ALIGN: RIGHT by ~{min(ALIGN_STEP_DEG, abs(angle)):.1f}° (error={angle:.2f}°)")
                    pivot(Motion.PIVOT_RIGHT, min(ALIGN_STEP_DEG, abs(angle)), cfg, left_motor_pwm, right_motor_pwm, ser)
                else:
                    print(f"ALIGN: LEFT by ~{min(ALIGN_STEP_DEG, abs(angle)):.1f}° (error={angle:.2f}°)")
                    pivot(Motion.PIVOT_LEFT,  min(ALIGN_STEP_DEG, abs(angle)), cfg, left_motor_pwm, right_motor_pwm, ser)
                time.sleep(0.02)
                continue  # re-check vision next loop

            # --- APPROACH: aligned; move forward in small chunks until success ---
            if not in_zone:
                if steps_taken >= MAX_APPROACH_STEPS:
                    print("Safety: reached max approach steps. Stopping.")
                    break
                print(f"APPROACH: forward {APPROACH_STEP_CM:.1f} cm (aligned within {ERROR_THRESHOLD_DEG}°)")
                drive_line_imu(Motion.FORWARD, APPROACH_STEP_CM, cfg, left_motor_pwm, right_motor_pwm, ser)
                steps_taken += 1
                time.sleep(0.02)
                continue  # re-check vision

            # --- SUCCESS: aligned and in zone → grasp ---
            print("IN POSITION: closing gripper.")
            close_gripper(gripper_pwm, cfg)

            # Back out slightly, settle, then drop
            if BACKUP_AFTER_GRASP_CM > 0:
                print(f"Backing out {BACKUP_AFTER_GRASP_CM:.1f} cm...")
                drive_line_imu(Motion.REVERSE, BACKUP_AFTER_GRASP_CM, cfg, left_motor_pwm, right_motor_pwm, ser)

            print(f"Settling for {DROP_SETTLE_S:.2f}s...")
            time.sleep(DROP_SETTLE_S)

            print("Opening gripper to drop.")
            open_gripper(gripper_pwm, cfg)

            # Optional: ease forward a touch so the gripper clears the object
            #if POST_DROP_FORWARD_CM > 0:
            #    print(f"Clearing forward {POST_DROP_FORWARD_CM:.1f} cm...")
            #    drive_line_imu(Motion.FORWARD, POST_DROP_FORWARD_CM, cfg, left_motor_pwm, right_motor_pwm, ser)

            print("Task complete.")
            time.sleep(1.0)
            break

    except KeyboardInterrupt:
        print("\nStopping loop.")
    finally:
        print("Cleaning up...")
        try:
            picam.stop()
        except Exception:
            pass
        left_motor_pwm.stop(); right_motor_pwm.stop()
        gripper_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
