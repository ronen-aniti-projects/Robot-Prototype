import RPi.GPIO as GPIO
import time
import numpy as np
import matplotlib.pyplot as plt
import serial

def read_heading(ser):
    if not ser or not ser.is_open:
        return None
    ser.write(b'R')
    line = ser.readline()
    if not line:
        return None
    try:
        # IMU provides heading from 0-359
        return int(line.decode(errors='ignore').strip())
    except (ValueError, UnicodeDecodeError):
        return None

def read_distance(cfg):
    distances_cm = []
    for _ in range(5):
        GPIO.output(cfg["sonar_trig"], True)
        time.sleep(0.00001)
        GPIO.output(cfg["sonar_trig"], False)

        start_time = time.time()
        stop_time = time.time()
        timeout = time.time() + 0.1 

        while GPIO.input(cfg["sonar_echo"]) == 0 and time.time() < timeout:
            start_time = time.time()
        while GPIO.input(cfg["sonar_echo"]) == 1 and time.time() < timeout:
            stop_time = time.time()
        
        if time.time() >= timeout: 
            continue

        duration = stop_time - start_time
        dist_cm = (duration * 34300) / 2
        if dist_cm < 400: 
            distances_cm.append(dist_cm)
        time.sleep(0.01)
    
    return sum(distances_cm) / len(distances_cm) if distances_cm else 400.0

def find_plateaus(data, min_length=5, tolerance=1.0):
    plateaus = []
    i = 0
    while i < len(data):
        start_index = i
        # Condition for plateau: Successive points about same value for a long enough stretch.
        
        # 1. Same Reading: A plateau point's successor has nearly the same value. 
        while (i + 1 < len(data) and abs(data[i+1] - data[start_index]) <= tolerance):
            i += 1
        
        # 2. Minimum Length: A plateau segment must be a minmum length. 
        if (i - start_index + 1) >= min_length:
            plateaus.append((start_index, i))
        
        i += 1
    return plateaus

if __name__ == "__main__":
    
    # Load the sonar pin numbers
    cfg = {
        "sonar_trig": 13,
        "sonar_echo": 11,
    }

    # Set up the sonar pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["sonar_trig"], GPIO.OUT)
    GPIO.setup(cfg["sonar_echo"], GPIO.IN)
    GPIO.output(cfg["sonar_trig"], False)
     
    # Set up the IMU (used for steering)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    time.sleep(2.0)
    ser.reset_input_buffer() 
    time.sleep(2)

    # Store data as (heading, distance) tuples
    collected_data = []
    print("Starting readings. Slowly spin the robot now.")
    print("Press Ctrl+C to stop and generate the plot.")

    try:
        while True:
            distance = read_distance(cfg)
            heading = read_heading(ser)

            collected_data.append((heading, distance))
    
            print(f"Heading: {heading} deg | Distance: {distance:.1f} cm")

            time.sleep(0.05) # Control the sampling rate

    except KeyboardInterrupt:
        pass 
    
    finally:        
        GPIO.cleanup()
        ser.close()


        headings, distances = zip(*sorted(collected_data))

        # 1. Smooth the distances: Reduce random fluctuations in distance measurements.
        #    Reason:  
        # Note: The "valid" option means only applying the convolution to elements bring full overlap with window
        window_size = 5 # Adjust
        smoothed_distances = np.convolve(distances, np.ones(window_size)/window_size, mode='valid')
        smoothed_headings = headings[:len(smoothed_distances)] 

        # 2. Detect Plateaus
        plateaus = find_plateaus(smoothed_distances, min_length=5, tolerance=2.0)

        plt.figure(figsize=(10, 6))
        # Plot original data with some transparency
        plt.plot(headings, distances, marker='.', linestyle='-', label='Raw Sonar Reading', alpha=0.3)
        # Plot smoothed data
        plt.plot(smoothed_headings, smoothed_distances, marker='o', linestyle='-', label='Smoothed Sonar Reading')
        
        # Plot the plateus, but also the original data and the smoothed data. 
        for start, end in plateaus:
            plt.plot(smoothed_headings[start:end+1], smoothed_distances[start:end+1], 
                        color='red', linewidth=4, label='Detected Wall (Plateau)' if start == plateaus[0][0] else "")

        plt.title('Manual 360-Degree Sonar Scan with Wall Detection')
        plt.xlabel('Heading (Degrees)')
        plt.ylabel('Distance (cm)')
        plt.grid(True)
        plt.legend()
        plt.show()
