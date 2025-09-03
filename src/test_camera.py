from picamera2 import Picamera2
from libcamera import Transform
import cv2
import time 
import os 
from pathlib import Path

base = Path(__file__).resolve().parent
out_dir = base / "data"
out_dir.mkdir(exist_ok=True)

picam = Picamera2() 
config = picam.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},  
    transform=Transform(vflip=True, hflip=True),
)
picam.configure(config)
picam.start() 


try: 
    while True:
        picam.capture_file(str(out_dir / "image.jpg"))
        print(f"Saved {out_dir / 'image.jpg'}")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass 
finally:
    picam.stop() 
    