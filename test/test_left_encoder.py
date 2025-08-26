import RPi.GPIO as GPIO
import time 
import json 

if __name__ == "__main__":    
    
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    
    MAX_TIME = 10.0
    MAX_COUNT = 100
    
    start_time = time.time() 
    count = 0
    
    try:
        prev_state = GPIO.input(cfg["left_encoder_pin"])
        while count < MAX_COUNT: 
            current_state = GPIO.input(cfg["left_encoder_pin"])
            if (current_state != prev_state):
                count += 1
                prev_state = current_state
            print(count)
            time.sleep(0.01)
    finally:
        print("Cleaning up")
        GPIO.cleanup()