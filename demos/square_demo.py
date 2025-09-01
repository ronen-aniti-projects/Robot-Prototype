import RPi.GPIO as GPIO
import time
import json
import numpy as np
from enum import Enum
import serial
import math

import matplotlib
# If running headless (no display), uncomment the next line:
# matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----------------- Enums & low-level helpers -----------------
class Motion(Enum):
    FORWARD = 1
    REVERSE = 2
    PIVOT_LEFT = 3
    PIVOT_RIGHT = 4

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
    """Map degrees to [-180, 180)."""
    return ((raw_angle + 180) % 360) - 180


# ----------------- Straight-line drive (your working style) -----------------
def drive_line_imu(mode, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser):
    set_logic(mode, cfg)

    ticks_per_rev = cfg["ticks_per_rev"]
    wheel_diameter_cm = cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm = np.pi * wheel_diameter_cm
    target_ticks = target_distance_cm / wheel_circumference_cm * ticks_per_rev

    prev_state_left = GPIO.input(cfg["left_encoder_pin"])
    prev_state_right = GPIO.input(cfg["right_encoder_pin"])
    count_left = 0
    count_right = 0

    base_heading = read_heading(ser)
    if base_heading is None:
        base_heading = 0

    LINE_BASE_DUTY = 55   # <<— modest power so you don't slam into corners
    KP_HEADING = 8        # simple trim

    while (count_left + count_right) / 2 < target_ticks:
        cur_left = GPIO.input(cfg["left_encoder_pin"])
        cur_right = GPIO.input(cfg["right_encoder_pin"])

        if cur_left != prev_state_left:
            count_left += 1
            prev_state_left = cur_left
        if cur_right != prev_state_right:
            count_right += 1
            prev_state_right = cur_right

        current_heading = read_heading(ser)
        if current_heading is None:
            current_heading = base_heading

        heading_error = wrap_angle(base_heading - current_heading)

        if heading_error < 0:
            adj = KP_HEADING * abs(heading_error)
            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY - adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY + adj, 0, 100))
            else:  # REVERSE
                left_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY + adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY - adj, 0, 100))
        elif heading_error > 0:
            adj = KP_HEADING * abs(heading_error)
            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY + adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY - adj, 0, 100))
            else:  # REVERSE
                left_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY - adj, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(LINE_BASE_DUTY + adj, 0, 100))
        else:
            left_motor_pwm.ChangeDutyCycle(LINE_BASE_DUTY)
            right_motor_pwm.ChangeDutyCycle(LINE_BASE_DUTY)

        time.sleep(0.001)

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)


# ----------------- Simple, high-torque pivot with tiny brake -----------------
def pivot_90(mode, cfg, left_motor_pwm, right_motor_pwm, ser):
    """
    Minimal pivot at full PWM with a small tolerance + brief counter-pulse brake.
    Keeps logic simple while preventing big overshoot.
    """
    assert mode in (Motion.PIVOT_LEFT, Motion.PIVOT_RIGHT)
    set_logic(mode, cfg)

    # Tunables (simple):
    FULL_DUTY = 100
    PIVOT_TOL_DEG = 3
    BRAKE_MS = 70
    BRAKE_DUTY = 40

    start = read_heading(ser)
    if start is None:
        start = 0

    if mode == Motion.PIVOT_LEFT:
        target = wrap_angle(start - 90)
    else:
        target = wrap_angle(start + 90)

    # Spin at full duty until within tolerance
    left_motor_pwm.ChangeDutyCycle(FULL_DUTY)
    right_motor_pwm.ChangeDutyCycle(FULL_DUTY)

    last_h = start
    while True:
        h = read_heading(ser)
        if h is None:
            h = last_h
        last_h = h

        err = wrap_angle(target - h)
        if abs(err) <= PIVOT_TOL_DEG:
            break
        time.sleep(0.01)

    # Hard stop
    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)

    # Brief counter-spin to kill inertia (very simple “brake”)
    if mode == Motion.PIVOT_LEFT:
        set_logic(Motion.PIVOT_RIGHT, cfg)
    else:
        set_logic(Motion.PIVOT_LEFT, cfg)

    left_motor_pwm.ChangeDutyCycle(BRAKE_DUTY)
    right_motor_pwm.ChangeDutyCycle(BRAKE_DUTY)
    time.sleep(BRAKE_MS / 1000.0)

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)


# ----------------- Tiny helpers for pose & plot -----------------
x, y, yaw = 0.0, 0.0, 0.0  # meters, radians

def norm_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def step_pose_forward(dist_m):
    global x, y
    x += dist_m * math.cos(yaw)
    y += dist_m * math.sin(yaw)

def step_pose_turn(deg):
    global yaw
    yaw = norm_angle(yaw + math.radians(deg))

def init_plot(side_m):
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_aspect("equal", adjustable="box")
    pad = 0.5
    L = side_m + pad
    ax.set_xlim(-pad, L)
    ax.set_ylim(-pad, L)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Estimated path (updates after each action)")
    ax.plot([0, side_m, side_m, 0, 0], [0, 0, side_m, side_m, 0], linestyle="--")
    line, = ax.plot([0.0], [0.0], marker="o")
    return fig, ax, line

def update_plot(fig, ax, line, xs, ys):
    line.set_data(xs, ys)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)


# ----------------- Main: 5 ft square, single 90° corners -----------------
if __name__ == "__main__":
    with open("config.json") as f:
        cfg = json.load(f)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_2"], GPIO.OUT)
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    GPIO.setup(cfg["right_encoder_pin"], GPIO.IN)

    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 1000); left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(cfg["right_motor_pwm"], 1000); right_motor_pwm.start(0)

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    time.sleep(2.0)
    ser.reset_input_buffer()

    # ~5 ft per side
    side_m = 1.524
    side_cm = side_m * 100.0

    # plotting state
    fig, ax, line = init_plot(side_m)
    xs, ys = [x], [y]

    try:
        for side in range(1, 5):
            # Move one side
            drive_line_imu(Motion.FORWARD, side_cm, cfg, left_motor_pwm, right_motor_pwm, ser)
            step_pose_forward(side_m)
            print(f"Moved {side_m:.3f} m FORWARD; pose (x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°)")
            xs.append(x); ys.append(y)
            update_plot(fig, ax, line, xs, ys)

            # Corner (skip after 4th side to end aligned)
            if side < 4:
                pivot_90(Motion.PIVOT_RIGHT, cfg, left_motor_pwm, right_motor_pwm, ser)
                step_pose_turn(90)
                print(f"Pivoted 90° RIGHT; pose (x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°)")
                xs.append(x); ys.append(y)
                update_plot(fig, ax, line, xs, ys)

        print("Square complete.")
        time.sleep(2)

    except KeyboardInterrupt:
        pass
    finally:
        print("Cleaning up")
        left_motor_pwm.stop()
        right_motor_pwm.stop()
        GPIO.cleanup()
        # If headless plotting, save the last frame:
        # plt.savefig("square_path.png")
