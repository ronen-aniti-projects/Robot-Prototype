import math
import numpy as np

# --- small helper: keep last good heading; tiny debounce ---
_last_hdg = 0
def read_heading_safe(ser, n=2):
    """
    Read heading n times, return circular mean in [0,360).
    Falls back to the last good value if reads are None.
    """
    global _last_hdg
    vals = []
    for _ in range(max(1, n)):
        ser.write(b'R')
        line = ser.readline()
        if line:
            try:
                vals.append(int(line.decode(errors='ignore').strip()))
            except ValueError:
                pass
        time.sleep(0.005)
    if vals:
        ang = np.deg2rad(vals)
        mean = math.degrees(math.atan2(np.mean(np.sin(ang)), np.mean(np.cos(ang))))
        hdg = (mean + 360.0) % 360.0
        _last_hdg = int(round(hdg))
    return _last_hdg

def pivot(mode, pivot_angle_deg, cfg, left_motor_pwm, right_motor_pwm, ser):
    """
    High-torque, simple pivot that hunts the target by detecting the error sign flip,
    then full-power counter-brakes. Designed for torque-limited drives.
    """
    assert mode in (Motion.PIVOT_LEFT, Motion.PIVOT_RIGHT)
    set_logic(mode, cfg)

    # --- Tunables for torque-limited rigs ---
    FULL_DUTY     = 100      # always spin at full torque
    COARSE_TOL    = 12       # deg; switch to zero-cross catch inside this band
    FINE_TOL      = 2        # deg; final accuracy
    STABLE_NEED   = 2        # consecutive in-tolerance reads to finish
    SETTLE_MS     = 40       # let IMU settle after stop/brake
    BRAKE_MS      = 140      # strong counter-brake (increase if still coasting)
    BREAKAWAY_MS  = 180      # minimum on-time to overcome static friction
    SPIN_STEP_MS  = 250      # max continuous spin chunk before re-check

    # Get start + target
    start = read_heading_safe(ser, n=4)
    if mode == Motion.PIVOT_LEFT:
        target = ((start - abs(int(pivot_angle_deg)) + 180) % 360) - 180
    else:
        target = ((start + abs(int(pivot_angle_deg)) + 180) % 360) - 180

    def err_deg(cur):
        # wrap to [-180, 180)
        return ((target - cur + 180) % 360) - 180

    # --- Phase 1: blast to coarse window ---
    left_motor_pwm.ChangeDutyCycle(FULL_DUTY)
    right_motor_pwm.ChangeDutyCycle(FULL_DUTY)
    while True:
        cur = read_heading_safe(ser, n=2)
        if abs(err_deg(cur)) <= COARSE_TOL:
            break
        time.sleep(0.01)
    # stop & settle
    left_motor_pwm.ChangeDutyCycle(0); right_motor_pwm.ChangeDutyCycle(0)
    time.sleep(SETTLE_MS / 1000.0)

    # --- Phase 2: zero-cross catch + brake loop ---
    stable = 0
    prev_err = err_deg(read_heading_safe(ser, n=3))

    # Safety cap on iterations to avoid infinite loop if sensor is bad
    for _ in range(20):
        cur = read_heading_safe(ser, n=3)
        e = err_deg(cur)

        if abs(e) <= FINE_TOL:
            stable += 1
            if stable >= STABLE_NEED:
                break
            time.sleep(SETTLE_MS / 1000.0)
            continue
        else:
            stable = 0

        # Choose direction toward target
        if e > 0:
            # need to rotate RIGHT
            set_logic(Motion.PIVOT_RIGHT, cfg)
        else:
            # need to rotate LEFT
            set_logic(Motion.PIVOT_LEFT, cfg)

        # Kick through static friction if we're not moving
        # (error not decreasing meaningfully)
        # We treat "not moving" as |e| >= |prev_err| - 1 deg
        if abs(e) >= abs(prev_err) - 1:
            on_ms = max(BREAKAWAY_MS, SPIN_STEP_MS)
        else:
            on_ms = SPIN_STEP_MS

        # Continuous spin but watch for error sign flip (zero-cross)
        left_motor_pwm.ChangeDutyCycle(FULL_DUTY)
        right_motor_pwm.ChangeDutyCycle(FULL_DUTY)

        t0 = time.time()
        started_sign = 1 if e > 0 else -1
        flipped = False
        while (time.time() - t0) * 1000.0 < on_ms:
            cur2 = read_heading_safe(ser, n=2)
            e2 = err_deg(cur2)
            sign_now = 1 if e2 > 0 else -1 if e2 < 0 else 0
            # if sign crossed (or hit exact 0), we reached/overshot target
            if sign_now == 0 or (sign_now != started_sign):
                flipped = True
                break
            time.sleep(0.01)

        # Stop spin
        left_motor_pwm.ChangeDutyCycle(0); right_motor_pwm.ChangeDutyCycle(0)
        time.sleep(SETTLE_MS / 1000.0)

        # If we flipped past target, apply strong counter-brake
        if flipped:
            # counter direction: opposite of the last spin
            if e > 0:
                # we were spinning RIGHT; brake LEFT
                set_logic(Motion.PIVOT_LEFT, cfg)
            else:
                # we were spinning LEFT; brake RIGHT
                set_logic(Motion.PIVOT_RIGHT, cfg)

            left_motor_pwm.ChangeDutyCycle(FULL_DUTY)
            right_motor_pwm.ChangeDutyCycle(FULL_DUTY)
            time.sleep(BRAKE_MS / 1000.0)
            left_motor_pwm.ChangeDutyCycle(0); right_motor_pwm.ChangeDutyCycle(0)
            time.sleep(SETTLE_MS / 1000.0)

        prev_err = e

    # Final stop (ensure motors are off)
    left_motor_pwm.ChangeDutyCycle(0); right_motor_pwm.ChangeDutyCycle(0)
