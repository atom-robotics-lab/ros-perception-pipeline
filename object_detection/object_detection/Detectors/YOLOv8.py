import cv2
from ultralytics import YOLO
import os

class YOLOv8:
  def __init__(self, model_dir_path, weight_file_name, conf_threshold = 0.7, score_threshold = 0.4, nms_threshold = 0.25):
    # Load model 
    self.model=YOLO(os.path.join(model_dir_path, weight_file_name))
    self.conf_threshold=conf_threshold
    self.predictions=[]
  
  def load_classes(self):

    self.class_list = []
    with open("scripts/utils/classes_v8.txt", "r") as f:
        self.class_list = [cname.strip() for cname in f.readlines()]
    return self.class_list

  # create list of dictionary containing predictions
  def create_predictions_list(self, class_ids, confidences, boxes):  
    
    for i in range(len(class_ids)):
        obj_dict = {
            "class_id": class_ids[i],
            "confidence": confidences[i],
            "box": boxes[i]
        }
        self.predictions.append(obj_dict)

  def  get_predictions(self,image):
    class_id=[]
    confidence=[]
    bb=[]
    result=self.model(image,self.conf_threshold) # Perform object detection on image
    row=result[0].boxes

    for box in row:
      class_id.append(box.cls)
      confidence.append(box.conf)
      bb.append(box.xyxy)
      
    self.create_predictions_list(class_id,confidence,bb)
    result=self.model(image,self.conf)
    bb_result=result[0].plot()                  # Frame with bounding boxes

    return self.predictions,bb_result
 