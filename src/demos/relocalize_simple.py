import RPi.GPIO as GPIO
import time
import numpy as np
import matplotlib.pyplot as plt
import serial

def read_heading(ser):
    """Requests and reads the current heading from the IMU via serial."""
    if not ser or not ser.is_open:
        return None
    ser.write(b'R')
    line = ser.readline()
    if not line:
        return None
    try:
        # IMU provides heading from 0-359, which is perfect for this plot
        return int(line.decode(errors='ignore').strip())
    except (ValueError, UnicodeDecodeError):
        return None

def read_distance(cfg):
    """Reads a single, averaged distance from the sonar sensor."""
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
    """
    Finds plateaus in a 1D array.
    A plateau is a sequence of points where the value is within a tolerance.
    Returns a list of (start_index, end_index) tuples for each plateau.
    """
    plateaus = []
    i = 0
    while i < len(data):
        start_index = i
        # Look for the end of a potential plateau
        while (i + 1 < len(data) and 
               abs(data[i+1] - data[start_index]) <= tolerance):
            i += 1
        
        # If the plateau is long enough, record it
        if (i - start_index + 1) >= min_length:
            plateaus.append((start_index, i))
        
        i += 1
    return plateaus

if __name__ == "__main__":
    cfg = {
        "sonar_trig": 13,
        "sonar_echo": 11,
    }

    # --- GPIO Setup ---
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(cfg["sonar_trig"], GPIO.OUT)
    GPIO.setup(cfg["sonar_echo"], GPIO.IN)
    GPIO.output(cfg["sonar_trig"], False)
    
    # --- Serial (IMU) Setup ---
    ser = None
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
        time.sleep(2.0)
        ser.reset_input_buffer() 
        print("IMU connection successful.")
    except serial.SerialException as e:
        print(f"Warning: Could not open serial port for IMU: {e}")
        print("Plot will use sample number instead of heading.")

    print("Waiting for sensor to settle...")
    time.sleep(2)

    # Store data as (heading, distance) tuples
    collected_data = []
    print("Starting readings. Slowly spin the robot now.")
    print("Press Ctrl+C to stop and generate the plot.")

    try:
        while True:
            distance = read_distance(cfg)
            heading = read_heading(ser)
            
            if heading is not None:
                collected_data.append((heading, distance))
                print(f"\rHeading: {heading}° | Distance: {distance:.1f} cm   ", end='')
            else:
                # Fallback if IMU fails: use sample number for x-axis
                collected_data.append((len(collected_data), distance))
                print(f"\rSample: {len(collected_data)} | Distance: {distance:.1f} cm   ", end='')

            time.sleep(0.05) # Control the sampling rate

    except KeyboardInterrupt:
        print("\nStopping readings.")

    finally:
        # --- Cleanup and Plotting ---
        print("Cleaning up and generating plot...")
        GPIO.cleanup()
        if ser and ser.is_open:
            ser.close()

        if collected_data:
            # Unzip the data into separate lists for plotting
            headings, distances = zip(*sorted(collected_data))

            # --- Data Smoothing ---
            window_size = 5 # Adjust for more or less smoothing
            smoothed_distances = np.convolve(distances, np.ones(window_size)/window_size, mode='valid')
            # Adjust headings to match the length of the smoothed data
            smoothed_headings = headings[:len(smoothed_distances)]

            # --- Plateau Detection ---
            # Find plateaus in the SMOOTHED data
            plateaus = find_plateaus(smoothed_distances, min_length=5, tolerance=2.0)

            plt.figure(figsize=(10, 6))
            # Plot original data with some transparency
            plt.plot(headings, distances, marker='.', linestyle='-', label='Raw Sonar Reading', alpha=0.3)
            # Plot smoothed data
            plt.plot(smoothed_headings, smoothed_distances, marker='o', linestyle='-', label='Smoothed Sonar Reading')
            
            # Highlight the detected plateaus
            for start, end in plateaus:
                plt.plot(smoothed_headings[start:end+1], smoothed_distances[start:end+1], 
                         color='red', linewidth=4, label='Detected Wall (Plateau)' if start == plateaus[0][0] else "")

            plt.title('Manual 360-Degree Sonar Scan with Wall Detection')
            plt.xlabel('Heading (Degrees)' if ser else 'Sample Number')
            plt.ylabel('Distance (cm)')
            plt.grid(True)
            plt.legend()
            # Set x-axis limits to be a full circle if using IMU
            if ser:
                plt.xlim(0, 360)
            plt.show()
        else:
            print("No data was collected.")
