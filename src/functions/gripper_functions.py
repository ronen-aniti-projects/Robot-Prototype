import RPi.GPIO as GPIO
import time 
import json 

def open_gripper(gripper_pwm, cfg):
    gripper_pwm.ChangeDutyCycle(cfg["gripper_duty_open"])
    time.sleep(1)
    return
    
def close_gripper(gripper_pwm, cfg): 
    gripper_pwm.ChangeDutyCycle(cfg["gripper_duty_close"])
    time.sleep(1)
    return
 
if __name__ == "__main__":   
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)
    
    # PWM Signal
    GPIO.setup(cfg["gripper_pwm"], GPIO.OUT)
    gripper_pwm = GPIO.PWM(cfg["gripper_pwm"], 50)
    
    # Start the gripper with duty for open
    gripper_pwm.start(cfg["gripper_duty_open"])
    time.sleep(1)
    
    try:
        
        while True:
            open_gripper(gripper_pwm, cfg)
            close_gripper(gripper_pwm, cfg)
            
    except KeyboardInterrupt:
        pass
    
    finally:
        gripper_pwm.stop()
        GPIO.cleanup()
        
    