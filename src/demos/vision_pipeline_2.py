from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np
import time

# ==== Define Demo Configuration ===
ROI_PCT = 0.20         # Percentage of top of frame to ignore
SUCCESS_ROI_PCT = 0.20 # Percentage of the bottom frame for "success" zone
HSV_LOW = np.array([95, 70, 40])     # Blue HSV mask low 
HSV_HIGH = np.array([130, 255, 255]) # Blue HSV high 
BASE_CAL_RATIO = 0.061 # Instructor provided pixle : angle ratio at 640 px x 480 px
SCALE_FACTOR = 2       # Demo uses 320 px x 240 px, so scale the ratio
MIN_AREA = 300         # Minimum required contour for object detection
ANGLE_TOLERANCE_DEGREES = 2.0    # Maximum absolute deviation requirement for being "in position"
SUCCESS_MASK_PIXEL_RATIO = 0.95  # Percent of masked pixels required to be in success zone for being "in position"

def main():
    """My Image Processing Pipeline Implemenation: With this, the robot """
    # Setup the Pi Camera Module:
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (320, 240), "format": "BGR888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)

    # Define a 3x3 kernel for morphological operations
    kernel = np.ones((3, 3), np.uint8)
    print("Press 'q' to quit.")

    try:
        while True:
            
            # 1. Raw Image Capture: Capture a raw image
            #    Reason: The image is the basic unit of analysis for this perception pipeline
            raw_frame = picam.capture_array()
            H, W, _ = raw_frame.shape
            roi_height = int(H * ROI_PCT)
            success_roi_y_start = H - int(H * SUCCESS_ROI_PCT)

            # 2. ROI Crop: Disregard the top portion of the image
            #    Reason: The top portion of the image is NOT ground plane and ONLY increases 
            #            the odds of false object detection. 
            processing_frame = raw_frame.copy()
            processing_frame[0:roi_height] = 0

            # 3. HSV Conversion: Convert the ROI cropped image to HSV from RGB. 
            #    Reason: HSV provides more robust means for object detection based on color
            hsv = cv2.cvtColor(processing_frame, cv2.COLOR_RGB2HSV)
            
            # 4. Blue Color Mask: Disregard all pixels that are not BLUE
            #    Reason: The 3D printed blocks are blue
            mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
            
            # 5. Morphological Operning: Remove mask "salt noise"
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 6. Morphological Closing: Fill small holes in mask objects
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 7. Contour Detection: Finds all contours in mask
            #    Reason: Contours are the outline of detected objects. Finding contours means 
            #            determining coordinate information regarding likely 3D printed blocks
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 8. Largest Contour Filtering: Extract the contour of the largest contour.
            #    Reason: The largest contour is likely to be the closest block to the robot 
            largest_contour = max(contours, key=cv2.contourArea) if contours else None

            # Create a new frame from the raw frame for annotations
            display_frame = raw_frame.copy()

            # Darken the rectangular "blacked out" region
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, 0), (W, roi_height), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)

            # Draw a dashed line at the top of the "success zone"
            dash_length = 10
            gap_length = 10
            for i in range(0, W, dash_length + gap_length):
                start_point = (i, success_roi_y_start)
                end_point = (i + dash_length, success_roi_y_start)
                cv2.line(display_frame, start_point, end_point, (0, 255, 255), 2)


            # Show the label "No target" if no block detected 
            angle_text = "No target"
            
            #  If the necessary size requirement has been met by the largest contour, then 
            #  draw a bounding box around the contour and compute the pixel error and pivot angle. 
            if largest_contour is not None and cv2.contourArea(largest_contour) > MIN_AREA:
                
                # 9. Bounding Box Drawing: Draw a bounding rectangle around the largest contour
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # 10. Pixel Offset Calculation: Compute the object's pixel offset
                image_center = (W // 2, H // 2)
                bb_center = (x + w // 2, y + h // 2)
                cv2.arrowedLine(display_frame, image_center, bb_center, (0, 255, 255), 2)  # Draw arrow to object

                # 11. Pivot Angle Estimation: Apply calibration ratio to determine pivot
                center_x = x + w / 2.0
                angle = CAL_RATIO * (center_x - W / 2.0)
                direction = "RIGHT" if angle >= 0 else "LEFT"
                angle_text = f"{direction} {abs(angle):.2f} DEGREES"

                # 12. Verify the Grasp Condition: Grasp only when both of the following are met:
                
                # Requirement 1: Alignment: Estimated pivot must be within tolerance of 0
                is_aligned = abs(angle) < ANGLE_TOLERANCE_DEGREES

                # Requirement 2: Estimated Proximity: Detected object exists only in success zone
                total_mask_pixels = cv2.countNonZero(mask)
                if total_mask_pixels > 0: # When some mask active:
                    # Only consider success zone
                    success_roi_mask = mask[success_roi_y_start:H, 0:W]
                    # Count total activate pixels in success zone
                    pixels_in_success_zone = cv2.countNonZero(success_roi_mask)
                    # Determine if all object is in success zone
                    pixel_ratio = pixels_in_success_zone / total_mask_pixels
                    is_in_zone = pixel_ratio >= SUCCESS_MASK_PIXEL_RATIO
                else: # When no mask active:
                    is_in_zone = False

                # Check the truth of both necessary conditions:
                if is_aligned and is_in_zone:
                    success_text = "IN POSITION"
                    text_size, _ = cv2.getTextSize(success_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
                    text_x = (W - text_size[0]) // 2
                    text_y = (H + text_size[1]) // 2
                    cv2.putText(display_frame, success_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (57, 255, 20), 2)


            # After checking, display the verdict: Is there a block in position? Is there a block at all? What's the pivot?
            cv2.putText(display_frame, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Show the Results for Demonstration: 
            display_frame_bgr = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("Annotated View", display_frame_bgr)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the Camera:
        print("Shutting down...")
        picam.stop()
        cv2.destroyAllWindows()
        
        
if __name__ == "__main__":
    main()
