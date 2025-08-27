import RPi.GPIO as GPIO
import time 
import json 
import numpy as np 
from enum import Enum 
import serial 

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

    # Assume pivot angle is positive 

    # Set the H-bridge for either forward or reverse straight line motion
    set_logic(mode, cfg)
    
    # Read the base heading before any motion
    base_heading   = read_heading(ser)

    # Set the P-controller parameters 
    base_duty   = 100
    kp          = 0.0
    
    # Read the IMU to record the start heading
    start_heading = read_heading(ser)
    
    # Determine the target heading from the pivot angle and requested direction
    if mode == Motion.PIVOT_LEFT:
        target_heading = wrap_angle(start_heading - pivot_angle_deg)
    elif mode == Motion.PIVOT_RIGHT:
        target_heading = wrap_angle(start_heading + pivot_angle_deg)
    
    error = wrap_angle(target_heading - read_heading(ser))

    while abs(error) > 0:
        adjustment = abs(error) * kp
        left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
        right_motor_pwm.ChangeDutyCycle(np.clip((base_duty + adjustment) * 0.95, 0, 100))
        error = wrap_angle(target_heading - read_heading(ser))
        time.sleep(0.005)

    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)
    

def drive_line_imu(mode, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser):

    # Set the H-bridge for either forward or reverse straight line motion
    set_logic(mode, cfg)
    
    # Determine how many encoder ticks are necessary to travel the provided distance
    ticks_per_rev            =  cfg["ticks_per_rev"]
    wheel_diameter_cm        =  cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm   =  np.pi * wheel_diameter_cm
    target_ticks             =  target_distance_cm / wheel_circumference_cm * ticks_per_rev
    
    # Read the initial state on the encoders
    prev_state_left    =   GPIO.input(cfg["left_encoder_pin"])
    prev_state_right   =   GPIO.input(cfg["right_encoder_pin"])

    # Start tick counters for both encoders
    count_left  = 0
    count_right = 0
    
    # Read the base heading before any motion
    base_heading   = read_heading(ser)

    # Set the P-controller parameters 
    base_duty   = 50
    kp          = 10
    
    # Track the encoder error and the heading error
    error         = 0           # left_ticks - right ticks
    heading_error = 0           # target_heading - heading

    # Drive the robot in a straight line until the target distance, as measured by encoder ticks,
    # has been reached, correcting direction along the way according to IMU heading error. 
    while (count_left + count_right) / 2 < target_ticks:
        
        # Read the current state of each encoder pin, then compare to the previous state to
        # determine if a state change has occured. 
        current_state_left    =   GPIO.input(cfg["left_encoder_pin"])
        current_state_right   =   GPIO.input(cfg["right_encoder_pin"])
        
        # If a state change on the left encoder pin has occured, increment the left encoder count. 
        if (current_state_left != prev_state_left):
            count_left += 1
            prev_state_left = current_state_left
        
        # If a state change on the right eencoder pin has occured, increment the right encoder count. 
        if (current_state_right != prev_state_right):
            count_right += 1
            prev_state_right = current_state_right

        # Read the current IMU heading angle 
        current_heading = read_heading(ser)

        # Compute the normalized difference in heading between the target heading and the current heading
        heading_error = wrap_angle(base_heading - current_heading)

        # If the target heading is to the left of the current heading, the robot should pivot right.
        if heading_error < 0:

            adjustment = kp * abs(heading_error)
            
            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adjustment, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
            
            if mode is Motion.REVERSE:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adjustment, 0, 100))
        
        # If the target heading is to the right of the current heading, the robot should pivot left.
        elif heading_error > 0:
            
            adjustment = kp * abs(heading_error)

            if mode is Motion.FORWARD:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adjustment, 0, 100))
            if mode is Motion.REVERSE:
                left_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adjustment, 0, 100))
                right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))

        # If the target heading is the same as the current heading, the robot should not pivot at all.
        else:
            left_motor_pwm.ChangeDutyCycle(np.clip(base_duty, 0, 100))
            right_motor_pwm.ChangeDutyCycle(np.clip(base_duty, 0, 100))

        # Set a 1 KHz speed limit on the control loop
        time.sleep(0.001)

    # Once 
    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)
    
if __name__ == "__main__":   
    
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)

    # Set the PWM pins to OUTPUT
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_pwm"], GPIO.OUT)
    
    # Set the LEFT motor logic pins to OUTPUT
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)

    # Set the RIGHT motor logic pins to OUTPUT
    GPIO.setup(cfg["right_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["right_motor_logic_2"], GPIO.OUT)

    # Set the LEFT and RIGHT encoder pins to INPUT
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    GPIO.setup(cfg["right_encoder_pin"], GPIO.IN)
    
    # Instantiate the PWM object on both motors
    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 400)
    left_motor_pwm.start(0)
    right_motor_pwm = GPIO.PWM(cfg["right_motor_pwm"], 400)
    right_motor_pwm.start(0)

    # Instantiate a serial object
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    time.sleep(2.0)
    ser.reset_input_buffer() 

    # Initialize a target distance for testing
    ticks_per_rev = cfg["ticks_per_rev"]
    wheel_diameter_cm = cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm =  np.pi * wheel_diameter_cm
    target_revs = 7
    target_distance_cm = wheel_circumference_cm * target_revs
    
    # Testing loop: Drive to target distance and repeat
    try:
        while True:
            
            #drive_line_imu(Motion.FORWARD, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser)
            #time.sleep(2)
            pivot(Motion.PIVOT_RIGHT, 90, cfg, left_motor_pwm, right_motor_pwm, ser)
            print("Complete")
            time.sleep(5)
            
            #drive_line_imu(Motion.REVERSE, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm, ser)
            #pivot(Motion.PIVOT_RIGHT, 90, cfg, left_motor_pwm, right_motor_pwm, ser)
            #print("Complete")
            #time.sleep(5)
    
    except KeyboardInterrupt:
        pass
    finally:
        print("Cleaning up")
        GPIO.cleanup()