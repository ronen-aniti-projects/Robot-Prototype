import serial
import time 

def read_heading():
    ser.write(b'R')
    line = ser.readline()
    if not line:
        return None 
    try:
        return int(line.decode(errors='ignore').strip())
    except ValueError:
        return None
    
if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.05)
    MAX_TIME = 5.0    # sec
    time.sleep(2.0)
    ser.reset_input_buffer() 
    
    heading = read_heading()
    if heading is not None:
        print("Heading: ", heading)
        