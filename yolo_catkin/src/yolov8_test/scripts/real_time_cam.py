import numpy as np
import cv2
from ultralytics import YOLO

model=YOLO("yolov8_test/model_notebook/version5.pt")

import requests
regions = ['mx', 'in'] 
cap=cv2.VideoCapture("/dev/video2")

while(cap.isOpened()):
    _,frame=cap.read()
    if _==True:
        res = model(frame,conf=0.70)
        res_plotted = res[0].plot(line_width=2)
        cv2.imshow("result", res_plotted)
            # Press Q on keyboard to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
  
# Break the loop
    else:
        break
  
# When everything done, release
# the video capture object
cap.release()
  
# Closes all the frames
cv2.destroyAllWindows()
