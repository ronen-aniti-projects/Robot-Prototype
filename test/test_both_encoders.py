import RPi.GPIO as GPIO
import time 
import json 

if __name__ == "__main__":    
    
    with open("config.json") as f:
        cfg = json.load(f)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["right_encoder_pin"], GPIO.IN)
    GPIO.setup(cfg["left_encoder_pin"], GPIO.IN)
    
    MAX_TIME = 10.0
    MAX_COUNT = 100
    
    start_time = time.time() 
    left_count = 0
    right_count = 0
    
    try:
        prev_state_left = GPIO.input(cfg["right_encoder_pin"])
        prev_state_right = GPIO.input(cfg["left_encoder_pin"])
        
        while left_count < MAX_COUNT and right_count < MAX_COUNT: 
            current_state_right = GPIO.input(cfg["right_encoder_pin"])
            current_state_left = GPIO.input(cfg["left_encoder_pin"])
            if (current_state_left != prev_state_left):
                left_count += 1
                prev_state_left = current_state_left
            if (current_state_right != prev_state_right):
                right_count += 1
                prev_state_right = current_state_right
            print(f"Right Count {right_count}, Left Count: {left_count}")
            time.sleep(0.01)
    finally:
        print("Cleaning up")
        GPIO.cleanup()