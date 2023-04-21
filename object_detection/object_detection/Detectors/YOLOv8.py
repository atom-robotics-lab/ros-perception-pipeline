import cv2
from ultralytics import YOLO

class Yolov8:
  def __init__(self):
    # Load model 
    self.model=YOLO("utils/version5.pt")
    self.conf=0.60

  def load_classes(self):
    self.class_list = []
    with open("scripts/utils/classes_v8.txt", "r") as f:
        self.class_list = [cname.strip() for cname in f.readlines()]
    return self.class_list

  def  wrapdetection(self,image):
    class_id=[]
    confidence=[]
    bb=[]
    result=self.model(image,self.conf)
    row=result[0].boxes
    for box in row:
      class_id.append(box.cls)
      confidence.append(box.conf)
      bb.append(box.xyxy)
    return class_id,confidence,bb

  def draw_bounding_box(self,image):
    result=self.model(image,self.conf)
    bb_result=result[0].plot()
    return bb_result
  
if __name__ == '__main__':
  detector=Yolov8()
  detector.wrapdetection()