from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time

# --- Configuration (Global Constants) ---
ROI_PCT = 0.20
HSV_LOW = np.array([95, 70, 40])   # WIDER/MORE TOLERANT
HSV_HIGH = np.array([130, 255, 255]) # WIDER/MORE TOLERANT
CAL_RATIO = 0.061 * 2 # adjust 
MIN_AREA = 300

def main():
    """Main function to run the camera and vision processing loop."""
    # --- Camera Setup ---
    #(640, 480)
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)

    # Kernel for morphological operations
    kernel = np.ones((3, 3), np.uint8)
    print("Press 'q' to quit.")

    try:
        # --- Main Loop ---
        while True:
            # 1) Capture a raw image
            raw_frame = picam.capture_array()
            H, W, _ = raw_frame.shape

            # Create a separate frame for processing where the ROI is blacked out
            processing_frame = raw_frame.copy()
            roi_height = int(H * ROI_PCT)
            processing_frame[0:roi_height] = 0

            # 2) Convert the processing frame to HSV and create a mask
            hsv = cv2.cvtColor(processing_frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 3) Find the largest contour in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea) if contours else None

            # --- Annotations on the Raw Frame ---
            display_frame = raw_frame.copy()

            # Create a transparent overlay for the ROI
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, 0), (W, roi_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

            angle_text = "No target"
            # 4) If a large enough contour is found, process and draw it
            if largest_contour is not None and cv2.contourArea(largest_contour) > MIN_AREA:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # --- NEW: Define center points for the arrow ---
                image_center = (W // 2, H // 2)
                bb_center = (x + w // 2, y + h // 2)

                # --- NEW: Draw the arrow from image center to target center ---
                cv2.arrowedLine(display_frame, image_center, bb_center, (0, 255, 255), 2)

                # Calculate pivot angle and determine direction text
                center_x = x + w / 2.0
                angle = CAL_RATIO * (center_x - W / 2.0)
                
                direction = "RIGHT" if angle >= 0 else "LEFT"
                # --- MODIFIED: Use "DEGREES" instead of a symbol ---
                angle_text = f"{direction} {abs(angle):.2f} DEGREES"

            # 5) Add text to the display frame
            cv2.putText(display_frame, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # --- Show Results ---
            cv2.imshow("Annotated View", display_frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # --- Cleanup ---
        print("Shutting down...")
        picam.stop()
        cv2.destroyAllWindows()
        
        
if __name__ == "__main__":
    main()