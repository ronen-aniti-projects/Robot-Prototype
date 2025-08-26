import RPi.GPIO as GPIO
import time
import json 

def read_distance():
    
    distances_cm = []
    is_danger = False
    
    for i in range(5):
        GPIO.output(cfg["sonar_trig"], True)
        time.sleep(0.00001)
        GPIO.output(cfg["sonar_trig"], False)

        while GPIO.input(cfg["sonar_echo"]) == 0:
            start = time.time()
        while GPIO.input(cfg["sonar_echo"]) == 1:
            end = time.time()

        duration = end - start
        dist_cm = (duration * 343.0) / 2 * 100
        distances_cm.append(dist_cm)
        time.sleep(0.01)
        
    average_dist_cm = sum(distances_cm) / len(distances_cm)

    if average_dist_cm < cfg["safety_distance_cm"]:
        is_danger = True
    
    return (is_danger, average_dist_cm)
    

if __name__ == "__main__":    
    
    with open("config.json") as f:
        cfg = json.load(f)
        
        
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["sonar_trig"], GPIO.OUT)
    GPIO.setup(cfg["sonar_echo"], GPIO.IN)
    GPIO.output(cfg["sonar_trig"], False)
        
    try:
        while True:
            is_danger, dist_cm = read_distance()
            if is_danger:
                print(f"DANGER: {dist_cm:.1f} cm")
            else: 
                print(f"{dist_cm:.1f} cm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()