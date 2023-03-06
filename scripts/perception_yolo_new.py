#!/usr/bin/env python

import time
import sys
import os

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionServer
from perception_pipeline.srv import find_object, find_objectResponse, find_objectRequest


# yolo constants add in ROS params
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.25

# to get path of model
ROOT_DIR = os.getcwd()

# bounding box colors
colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

# is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"


class WorkpieceDetector :

    def __init__(self):
        # calculate fps, TODO: create a boolean to enable/diable show_fps
        self.frame_count = 0
        self.total_frames = 0
        self.fps = -1
        self.start = time.time_ns()
        #self.frame = frame
        self.bridge = CvBridge()
        #self.object = ob_name
        self.bb_frame = None
        self.frame = None
        self.flag = 0
        self.return_value = False
        self.objectid = None
        self.model_dir_path = rospy.get_param("model_params/model_dir_path")
        print(rospy.get_param("model_params/model_name"))
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.load_capture)
        # maybe return final image with bounding box and fps instead of imshow
        self.yolo_service = rospy.Service('yolo_service', find_object, self.find_object_cb)
        
    # yolo service callback that returns boolean True if object was detected, it calls the main control loop
    def find_object_cb(self, req) :
       self.object = req.object_name
       rospy.loginfo("Perception request received for {}".format(self.object))
       result = self.control_loop()
       print("YOLO Result: {}".format(result))

       return find_objectResponse(self.return_value)
    
    # load model and prepare its backend to either run on GPU or CPU, see if it can be added in constructor
    def build_model(self , is_cuda):
        # TODO: use from ros param or launch param
        self.net = cv2.dnn.readNet("src/perception_pipeline/scripts/utils/auto_final.onnx")
        if is_cuda:
            print("Attempt to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self,image):
        # convert image to 640x640 
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds

    def load_capture(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data)

    # load classes.txt that contains mapping of model with labels
    def load_classes(self):
        self.class_list = []
        with open("src/perception_pipeline/scripts/utils/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list

    # extract bounding box, class IDs and confidences of detected objects
    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor =  image_height / INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= CONFIDENCE_THRESHOLD:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        # removing intersecting bounding boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    # makes image square with dimension max(h, w)
    def format_yolov5(self):
        row, col, _ = self.frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = self.frame
        return result   

    def display_objects(self) :
        
        if self.bb_frame is not None :
            cv2.imshow("Object Detection", self.bb_frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                return

        else:
            rospy.loginfo("Bounding box frame is None")


    def control_loop(self) :
        
        if self.frame is None :
            print("Frame is None")
            # wait or possibly return?
        else:
            # load model
            self.build_model(False)
            
            # load classes (object labels)
            self.load_classes()

            # get index of object name
            self.objectid = self.class_list.index(self.object)

            # make image square
            inputImage = self.format_yolov5()

            outs = self.detect(inputImage)
            class_ids, confidences, boxes = self.wrap_detection(inputImage, outs[0])

            self.frame_count += 1
            self.total_frames += 1
                            
            print("Detected ids : ", class_ids)
            
            # if requested object is found
            if self.objectid in class_ids :
                print("Item {} found".format(self.object))
                index = class_ids.index(self.objectid)
                box = boxes[index]
                print(box)
                self.return_value = True
            else:
                print("Item {} not found".format(self.object))
                self.return_value = False
            
            # draw bounding box and add label
            for (classid, confidence, box) in zip(class_ids, confidences, boxes):
                color = colors[int(classid) % len(colors)]
                cv2.rectangle(self.frame, box, color, 2)
                cv2.rectangle(self.frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                try :
                    cv2.putText(self.frame, self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                except :
                    pass
            
            # fps
            if self.frame_count >= 30:
                self.end = time.time_ns()
                self.fps = 1000000000 * frame_count / (self.end - self.start)
                self.frame_count = 0
                self.start = time.time_ns()
            if self.fps > 0:
                self.fps_label = "FPS: %.2f" % self.fps
                cv2.putText(self.frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)           

            cv2.imshow("yolo_output", self.frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            return self.return_value


if __name__ == "__main__" :
    rospy.init_node("perception_yolo")

    wd = WorkpieceDetector()
    rospy.spin()
    
    

