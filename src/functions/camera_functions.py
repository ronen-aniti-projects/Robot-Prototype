from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time
import json 

def confirm_grip(picam):
    # Take RGB picture
    
    # Mask on target color threshold 
    
    # If most bottom 1/10 of pixels are activated, return True, else return False.
      
    pass

def take_picture(picam):
    rgb = picam.capture_array()
    return rgb

def apply_mask(rgb):
    # Returns a mask of the target color
    pass 

def confirm_block(mask):
    # Returns true only if bounding box area is not at least some threshold amount 
    pass

def estimate_pivot(mask):
    # Read masked frame
    
    # Identify the center x of the bounding box of the largest contour
    
    # Declare error is the center x of the image - the bounding box center x
    
    # if the error is negative that means pivot right else left
    
    # take the abs of the error
    
    # multiple the abs of the error by the angle to pixel ratio
    
    # return the pivot angle and the string direction
    
    pass


if __name__ == "__main__":
    
    # To forward graphics: ssh -X raniti@192.168.86.242
    with open("config.json") as f:
        cfg = json.load(f)
        
    picam = Picamera2() 
    config = picam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},  
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start() 

    try: 
        while True:
            rgb = take_picture(picam)
            print(f"Captured image shape: {rgb.shape}, dtype: {rgb.dtype}")
            
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("Camera View", bgr)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass 
    finally:
        picam.stop() 
        