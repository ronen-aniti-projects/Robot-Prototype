#!/usr/bin/env python3
"""
Dense Farnebäck + RANSAC Homography on ROI (crop-based)
- Homography is estimated mapping ROI(t) -> ROI(t+Δt) in ROI-local coordinates.
- Dense outliers are computed ONLY inside the ROI.
- Output is a binary mask (white=outlier in ROI, black elsewhere).

Display:
  Left  : Original (BGR) with ROI rectangle
  Right : Binary outlier mask pasted into full image (ROI-only)

Keys:
  q : quit

Headless (no DISPLAY): saves 'dense_outliers_roi_binary.png' each loop.
"""

import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

# ---------------- Camera ----------------

def take_picture(picam):
    """Capture one RGB frame as a NumPy array (H, W, 3), uint8."""
    return picam.capture_array()

# ------------- ROI: bottom third (y), middle half (x) -------------

def roi_rect_from_hw(h, w):
    """
    ROI = bottom third in height and middle half in width.
    Returns (x0, y0, x1, y1) with x1,y1 exclusive.
    """
    y0, y1 = (2 * h) // 3, h
    x0, x1 = 0, w
    return x0, y0, x1, y1

# ---- Dense flow + RANSAC H in ROI-local coords -> binary mask (ROI only) ----

def compute_outlier_mask_dense_roi_cropped(
    frame1_rgb, frame2_rgb,
    ransac_sample_count=5000,    # how many ROI pixels to sample for H
    ransac_reproj_thresh=3.0,    # px in ROI coords
    outlier_thresh_px=2.0        # per-pixel error threshold in ROI coords
):
    """
    Returns:
      full_mask (H,W) uint8: 255 = outlier inside ROI, 0 elsewhere
    The homography maps ROI(t) -> ROI(t+Δt) using ROI-local coordinates.
    """
    h, w = frame1_rgb.shape[:2]
    x0, y0, x1, y1 = roi_rect_from_hw(h, w)

    # Crop ROI from both frames (ROI-local images)
    f1_roi = frame1_rgb[y0:y1, x0:x1]
    f2_roi = frame2_rgb[y0:y1, x0:x1]

    # Convert ROI crops to gray
    g1 = cv2.cvtColor(f1_roi, cv2.COLOR_RGB2GRAY)
    g2 = cv2.cvtColor(f2_roi, cv2.COLOR_RGB2GRAY)

    rh, rw = g1.shape  # ROI height/width

    # Dense Farnebäck flow on ROI crops ONLY
    flow = cv2.calcOpticalFlowFarneback(
        g1, g2, None,
        pyr_scale=0.5, levels=3, winsize=15,
        iterations=3, poly_n=5, poly_sigma=1.2, flags=0
    )
    vx, vy = flow[..., 0], flow[..., 1]

    # Build ROI-local correspondences
    xs, ys = np.meshgrid(np.arange(rw, dtype=np.float32),
                         np.arange(rh, dtype=np.float32))
    p1_all = np.stack([xs, ys], axis=-1).reshape(-1, 2)            # (N,2) in ROI coords
    p2_all = np.stack([xs + vx, ys + vy], axis=-1).reshape(-1, 2)  # (N,2) in ROI coords

    # Sample for RANSAC (in ROI only)
    N = p1_all.shape[0]
    samp = min(ransac_sample_count, N)
    if samp < 8:
        # Not enough points to estimate H
        full_mask = np.zeros((h, w), np.uint8)
        return full_mask

    idx = np.random.choice(N, size=samp, replace=False)
    p1_s = p1_all[idx]
    p2_s = p2_all[idx]

    # Fit homography H_roi: maps ROI(t) -> ROI(t+Δt) in ROI-local coords
    H, _ = cv2.findHomography(p1_s, p2_s, cv2.RANSAC, ransac_reproj_thresh)
    if H is None:
        # Fall back to no mask if H fails
        full_mask = np.zeros((h, w), np.uint8)
        return full_mask

    # Predict motion for every ROI pixel via H and compute reprojection error
    ones = np.ones((N, 1), dtype=np.float32)
    proj = np.hstack([p1_all.astype(np.float32), ones]) @ H.T   # (N,3)
    proj /= np.clip(proj[:, 2:3], 1e-6, None)
    err = np.linalg.norm(p2_all - proj[:, :2], axis=1).reshape(rh, rw)  # ROI-local error

    # Binary outlier mask in ROI coords
    roi_mask_bin = (err > outlier_thresh_px).astype(np.uint8) * 255  # (rh,rw)

    # Paste ROI mask back into full image-sized mask
    full_mask = np.zeros((h, w), np.uint8)
    full_mask[y0:y1, x0:x1] = roi_mask_bin
    return full_mask

# ---------------- Main ----------------

def main():
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        transform=Transform(vflip=True, hflip=True),
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)  # warm-up

    has_display = bool(os.environ.get("DISPLAY"))
    prev_rgb = take_picture(picam)

    try:
        while True:
            
            rgb = take_picture(picam)
            mask = compute_outlier_mask_dense_roi_cropped(
                prev_rgb, rgb,
                ransac_sample_count=5000,
                ransac_reproj_thresh=3.0,
                outlier_thresh_px=2.0
            )
            prev_rgb = rgb

            # Left: original with ROI box
            left_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            h, w = left_bgr.shape[:2]
            x0, y0, x1, y1 = roi_rect_from_hw(h, w)
            cv2.rectangle(left_bgr, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 255), 2, lineType=cv2.LINE_AA)
            cv2.putText(left_bgr, "Original (ROI outlined)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            # Right: binary mask (grayscale → BGR for side-by-side)
            right_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(right_bgr, "RANSAC outliers (binary, ROI-only)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            composite = cv2.hconcat([left_bgr, right_bgr])

            if has_display:
                cv2.imshow("Original | ROI Outliers (Binary)", composite)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                cv2.imwrite("dense_outliers_roi_binary.png", composite)
                time.sleep(0.25)

    except KeyboardInterrupt:
        pass
    finally:
        picam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # To forward graphics from the Pi:
    #   ssh -X ronen@<pi-ip>
    main()
