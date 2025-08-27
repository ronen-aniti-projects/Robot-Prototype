from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time
import json 
import datetime
import os


def take_picture(picam):
    rgb = picam.capture_array()
    return rgb


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

    
    save_dir = "src/functions/data"

    try: 
        while True:
            rgb = take_picture(picam)
            print(f"Captured image shape: {rgb.shape}, dtype: {rgb.dtype}")
            
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("Camera View", bgr)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break 
            elif key == ord('x'):
                
                filename = os.path.join(
                    save_dir, 
                    f"snapshot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                )
                cv2.imwrite(filename, bgr)
                print(f"Saved {filename}")
            
            time.sleep(0.05)  
    except KeyboardInterrupt:
        pass 
    finally:
        picam.stop() 
        cv2.destroyAllWindows()
