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


# ----------------- Your existing bits (unchanged shape) -----------------
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
    return ((raw_angle + 180) % 360) - 180

def pivot(mode, pivot_angle_deg, cfg, left_motor_pwm, right_motor_pwm, ser):
    # Minimal “do the thing”; we’ll use commanded angle for pose update.
    set_logic(mode, cfg)

    base_duty = 100
    kp = 0.0

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
    while abs(error) > 0:
        adjustment = abs(error) * kp
        left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
        right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
        error = wrap_angle(target_heading - (read_heading(ser) or start_heading))

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)

def drive_line_imu(mode, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser):
    # Minimal straight-line with IMU trim; we’ll use commanded distance for pose update.
    set_logic(mode, cfg)

    ticks_per_rev = cfg["ticks_per_rev"]
    wheel_diameter_cm = cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm = np.pi * wheel_diameter_cm
    target_ticks = target_distance_cm / wheel_circumference_cm * ticks_per_rev

    prev_state_left = GPIO.input(cfg["left_encoder_pin"])
    prev_state_right = GPIO.input(cfg["right_encoder_pin"])
    count_left = 0
    count_right = 0

    base_heading = read_heading(ser) or 0
    base_duty = 100
    kp = 10

    while (count_left + count_right) / 2 < target_ticks:
        cur_left = GPIO.input(cfg["left_encoder_pin"])
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


# ----------------- Tiny helpers for pose & plot (barebones) -----------------
# Global-ish pose state (meters, radians)
x, y, yaw = 0.0, 0.0, 0.0

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
    # intended square
    ax.plot([0, side_m, side_m, 0, 0], [0, 0, side_m, side_m, 0], linestyle="--")
    line, = ax.plot([0.0], [0.0], marker="o")
    return fig, ax, line

def update_plot(fig, ax, line, xs, ys):
    line.set_data(xs, ys)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)


# ----------------- Main: 5 ft square with 10° corner pivots -----------------
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
    corner_step_deg = 10

    # plotting state
    fig, ax, line = init_plot(side_m)
    xs, ys = [x], [y]

    try:
        for side in range(1, 5):
            # move one side
            drive_line_imu(Motion.FORWARD, side_cm, cfg, left_motor_pwm, right_motor_pwm, ser)
            step_pose_forward(side_m)
            print(f"Moved {side_m:.3f} m FORWARD; new pose (x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°)")
            xs.append(x); ys.append(y)
            update_plot(fig, ax, line, xs, ys)

            # corner (skip after 4th side if you want to stop aligned)
            if side < 4:
                for _ in range(0, 90, corner_step_deg):
                    pivot(Motion.PIVOT_RIGHT, corner_step_deg, cfg, left_motor_pwm, right_motor_pwm, ser)
                    step_pose_turn(corner_step_deg)  
                    print(f"Pivoted {corner_step_deg}° RIGHT; new pose (x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°)")
                    # small dot at corner progress (optional)
                    xs.append(x); ys.append(y)
                    update_plot(fig, ax, line, xs, ys)

        print("Square complete.")
        time.sleep(3)

    except KeyboardInterrupt:
        pass
    finally:
        print("Cleaning up")
        left_motor_pwm.stop()
        right_motor_pwm.stop()
        GPIO.cleanup()
        # If headless plotting, you can save the last frame:
        # plt.savefig("square_path.png")
