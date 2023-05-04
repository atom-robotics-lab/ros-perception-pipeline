import tensorflow_hub as hub
import cv2
import numpy
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt

import tempfile


# For drawing onto the image.
import numpy as np
from PIL import Image
from PIL import ImageColor
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageOps

# For measuring the inference time.
import time 

class EfficientDet:
    def __init__(self, model_dir_path, weight_file_name, conf_threshold = 0.7, score_threshold = 0.25, nms_threshold = 0.4):
        
        self.frame_count = 0
        self.total_frames = 0
        self.fps = -1
        self.start = time.time_ns()
        
        self.model_dir_path = model_dir_path
        self.weight_file_name = weight_file_name
        self.conf=conf_threshold
        
        # Resizing image
        self.img_height=800
        self.img_width=800
        self.predictions=[]

        self.build_model()
        self.load_classes()

    def build_model(self) :
        module_handle="https://tfhub.dev/tensorflow/efficientdet/d0/1"
        # Loading model directly from TensorFlow Hub
        self.detector = hub.load(module_handle)


    def load_classes(self):
        self.labels = []
        with open(self.model_dir_path + "/classes.txt", "r") as f:
            self.labels = [cname.strip() for cname in f.readlines()]
        return self.labels

    def display_image(self,image):
        cv2.imshow("result", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def draw_bounding_box_on_image(self,image,ymin,xmin,ymax,xmax,color,font,thickness=4,display_str_list=()):
        """Adds a bounding box to an image."""
        draw = ImageDraw.Draw(image)
        im_width, im_height = image.size
        (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                        ymin * im_height, ymax * im_height)
        draw.line([(left, top), (left, bottom), (right, bottom), (right, top),
                    (left, top)],
                    width=thickness,
                    fill=color)
        # If the total height of the display strings added to the top of the bounding
        # box exceeds the top of the image, stack the strings below the bounding box
        # instead of above.
        display_str_heights = [font.getsize(ds)[1] for ds in display_str_list]
        # Each display_str has a top and bottom margin of 0.05x.
        total_display_str_height = (1 + 2 * 0.05) * sum(display_str_heights)

        if top > total_display_str_height:
            text_bottom = top
        else:
            text_bottom = top + total_display_str_height
        # Reverse list and print from bottom to top.
        for display_str in display_str_list[::-1]:
            text_width, text_height = font.getsize(display_str)
            margin = np.ceil(0.05 * text_height)
            draw.rectangle([(left, text_bottom - text_height - 2 * margin),
                            (left + text_width, text_bottom)],
                        fill=color)
            draw.text((left + margin, text_bottom - text_height - margin),
                    display_str,
                    fill="black",
                    font=font)
            text_bottom -= text_height - 2 * margin

    # create list of dictionary containing predictions
    def create_predictions_list(self, class_ids, confidences, boxes):
        for i in range(len(class_ids)):
            obj_dict = {
                "class_id": class_ids[i],
                "confidence": confidences[i],
                "box": boxes[i]
            }
            self.predictions.append(obj_dict)

    def draw_boxes(self,image,boxes,class_ids,confidences,max_boxes=10):
        """Overlay labeled boxes on an image with formatted scores and label names."""
        colors = list(ImageColor.colormap.values())

        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Regular.ttf",
                                    25)
        except IOError:
            print("Font not found, using default font.")
            font = ImageFont.load_default()

        for i in range(min(boxes.shape[0], max_boxes)):
            if confidences[i] >= self.conf:
                ymin, xmin, ymax, xmax = tuple(boxes[i])
                display_str = "{}: {}%".format(self.labels[class_ids[i]],
                                                int(100 * confidences[i]))
                color = colors[hash(class_ids[i]) % len(colors)]
                image_pil = Image.fromarray(np.uint8(image)).convert("RGB")
                self.draw_bounding_box_on_image(image_pil,ymin,xmin,ymax,xmax,color,font,display_str_list=[display_str])
                np.copyto(image, np.array(image_pil))
        return image

    def load_img(self,path):
        img = tf.io.read_file(path)
        img = tf.image.decode_jpeg(img, channels=3)
        return img

    def get_predictions(self,cv_image):

        #Convert img to RGB
        frame = cv_image.copy()

        self.frame_count += 1
        self.total_frames += 1

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # COnverting to uint8
        rgb_tensor = tf.convert_to_tensor(rgb, dtype=tf.uint8)

        #Add dims to rgb_tensor
        rgb_tensor = tf.expand_dims(rgb_tensor , 0)

        start_time = time.time()
        result = self.detector(rgb_tensor)
        end_time = time.time()

        result = {key:value.numpy() for key,value in result.items()}
        
        #print("Found %d objects." % len(result["detection_scores"]))
        #print("Inference time: ", end_time-start_time)
        
        self.create_predictions_list(result["detection_boxes"][0],result["detection_classes"][0], result["detection_scores"][0])
        #image_with_boxes = self.draw_boxes(cv_image,result["detection_boxes"][0],result["detection_classes"][0], result["detection_scores"][0])
        
        # fps
        if self.frame_count >= 30:
            self.end = time.time_ns()
            self.fps = 1000000000 * self.frame_count / (self.end - self.start)
            self.frame_count = 0
            self.start = time.time_ns()
        
        if self.fps > 0:
            self.fps_label = "FPS: %.2f" % self.fps
            cv2.putText(frame, self.fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)           
            
        return [self.predictions, frame]


    def detect_img(self,image_url):
        start_time = time.time()
        self.run_detector(self.detector, image_url)#Convert img to RGB
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # COnverting to uint8
        rgb_tensor = tf.convert_to_tensor(rgb, dtype=tf.uint8)
        #Add dims to rgb_tensor
        rgb_tensor = tf.expand_dims(rgb_tensor , 0)
        start_time = time.time()
        result = self.detector(rgb_tensor)
        end_time = time.time()
        result = {key:value.numpy() for key,value in result.items()}
        print("Found %d objects." % len(result["detection_scores"]))
        print("Inference time: ", end_time-start_time)
        self.create_predictions_list(cv_image,result["detection_boxes"][0],result["detection_classes"][0], result["detection_scores"][0])
        image_with_boxes = self.draw_boxes(cv_image,result["detection_boxes"][0],result["detection_classes"][0], result["detection_scores"][0])
        self.display_image(self.predictions,image_with_boxes)

        end_time = time.time()
        print("Inference time:",end_time-start_time)

if __name__=='__main__':
  # Load model 
    det = EfficientDet()
    det.detect_img("/home/sanchay/yolo_catkin/src/yolov8_test/scripts/dog_cat.jpg")