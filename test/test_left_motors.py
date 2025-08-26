import RPi.GPIO as GPIO
import time 
import json 

if __name__ == "__main__":   
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)
    
    # Right Motors
    GPIO.setup(cfg["left_motor_logic_1"], GPIO.OUT)
    GPIO.setup(cfg["left_motor_logic_2"], GPIO.OUT)
    GPIO.output(cfg["left_motor_logic_1"], GPIO.LOW)
    GPIO.output(cfg["left_motor_logic_2"], GPIO.HIGH)

    # PWM Signal
    GPIO.setup(cfg["left_motor_pwm"], GPIO.OUT)
    
    left_motor_pwm = GPIO.PWM(cfg["left_motor_pwm"], 1000)
    left_motor_pwm.start(0)
    
    try:
        while True:
            for dc in range(0, 101, 5):
                left_motor_pwm.ChangeDutyCycle(dc)
                time.sleep(0.4)
            for dc in range(100, -1, -5):
                left_motor_pwm.ChangeDutyCycle(dc)
                time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    left_motor_pwm.stop()
    GPIO.cleanup()