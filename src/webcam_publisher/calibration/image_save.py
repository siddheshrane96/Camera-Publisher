import cv2
import numpy as np
import yaml
import os
import time

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print(f"Error: could not open webcam")
    exit()

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)

output_path = "/home/siddhesh/Downloads/camera_scripts/calibration_frames"

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    # print(f"Frame shape {frame.shape}")
    cv2.imshow("Webcam", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') or key == ord('S'):
        print(f"Frame shape {frame.shape}")
        time_now = time.time_ns() // 10e6
        file_name = str(int(time_now)) + ".png"
        output_img = os.path.join(output_path, file_name)
        cv2.imwrite(output_img, frame)
        print("Pressed S")
    elif key == ord('q') or key == ord('Q'):
        print("Quitting")
        break
cap.release()
cv2.destroyAllWindows()