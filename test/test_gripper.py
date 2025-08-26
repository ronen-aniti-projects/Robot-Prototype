import RPi.GPIO as GPIO
import time 
import json 

if __name__ == "__main__":   
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)
    
    # PWM Signal
    GPIO.setup(cfg["gripper_pwm"], GPIO.OUT)
    
    gripper_pwm = GPIO.PWM(cfg["gripper_pwm"], 50)
    gripper_pwm.start(0)
    
    try:
        
        while True:
            for dc_scaled in range(35, 116, 1):
                gripper_pwm.ChangeDutyCycle(dc_scaled / 10.0)
                time.sleep(0.1)
            for dc_scaled in range(115, 34, -1):
                gripper_pwm.ChangeDutyCycle(dc_scaled / 10.0)
                time.sleep(0.1)
            gripper_pwm.ChangeDutyCycle(7.5); time.sleep(1)   
        
    except KeyboardInterrupt:
        pass
    gripper_pwm.stop()
    GPIO.cleanup()
    
    