#!/usr/bin/env python

from inspect import FrameInfo
import cv2
import time
import sys
#from defer import return_value
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, CompressedImage
import os
import rospy

from geometry_msgs.msg import PoseStamped
from object_msgs.msg import ObjectPose
from sensor_msgs.msg import Image
# from find_object_2d.msg import ObjectsStamped, DetectionInfo

from ebot_perception.srv import *

from actionlib import SimpleActionServer
#from ebot_handler.msg import PerceptionAction, PerceptionResult, PerceptionFeedbacks
from rospy.exceptions import ROSException

import roslaunch

import message_filters


INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.25

ROOT_DIR = os.getcwd()


colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

#net = build_model(is_cuda)
#capture = load_capture()

start = time.time_ns()
frame_count = 0
total_frames = 0
fps = -1



class WorkpieceDetector :

    def __init__(self):
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
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2/compressed", CompressedImage, self.load_capture)
        self.yolo_service = rospy.Service('yolo_service', find_object, self.find_object_cb)
        
    def find_object_cb(self, req) :
       self.object = req.object_name
       rospy.loginfo("Perception request received for {}".format(self.object))
       result = self.control_loop()
       print("YOLO Result: {}".format(result))

       return find_objectResponse(self.return_value)
    
    def build_model(self , is_cuda):
        self.net = cv2.dnn.readNet("src/automate-robot-bvp/ebot_perception/scripts/utils/auto_final.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return self.net

    def detect(self,image,net ):
        self.net = net
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds

    def load_capture(self, data):
        
        #self.capture = cv2.VideoCapture("sample.mp4")

        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.frame = image_np
        self.capture = self.frame        

        # if self.flag == 0 and self.frame is not None:
        #     #self.control_loop()
        #     self.flag = 1
        # else:
        #     print("Control Loop ran once")


    def load_classes(self):
        self.class_list = []
        with open("src/automate-robot-bvp/ebot_perception/scripts/utils/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list



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

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_yolov5(self,frame):

        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result   

    def display_objects(self) :
        
        if self.bb_frame is not None :
            cv2.imshow("Object Detection", self.bb_frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                return

        else:
            rospy.loginfo("Bounding box frame is None")


    def control_loop(self) :
        
        rospy.loginfo("Waiting 3 secs for frame")
        #rospy.spin(3)

        

        if self.frame is None :
            print("Frame is None")

        else:
            
        
            self.net = self.build_model(is_cuda)
            self.load_classes()
            self.objectid = self.class_list.index(self.object)
            #self.capture = self.load_capture()

                #while self.capture is not None:
            #frame = self.capture
            cv2.imwrite("frame.png", self.frame)
            print("control_loop")

            if self.frame is None:
                print("End of stream")
                exit()

                
            
            
            inputImage = self.format_yolov5(self.frame)
            #resized = cv2.resize(inputImage , (640,640))
            #blurred = cv2.blur(resized ,(10,10))
            outs = self.detect(inputImage, self.net)
            class_ids, confidences, boxes = self.wrap_detection(inputImage, outs[0])
            #print("ID : " , class_ids)
            #print("Boxes : ",boxes)
            self.frame_count += 1
            self.total_frames += 1
            self.objectid = self.class_list.index(self.object)
                            
            print("Detected boxes : ", boxes)
            print("Detected ids : ", class_ids)
            
            if self.objectid in class_ids :
                print("Item {} found".format(self.object))
                index = class_ids.index(self.objectid)
                box = boxes[index]
                
                
                print(box)
                
                self.return_value = True
            else:
                print("Item {} not found".format(self.object))
                self.return_value = False
            
            for (classid, confidence, box) in zip(class_ids, confidences, boxes):
                color = colors[int(classid) % len(colors)]
                cv2.rectangle(self.frame, box, color, 2)
                cv2.rectangle(self.frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                try :
                    cv2.putText(self.frame, self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                except :
                    pass
            
            #self.return_value=True
            #else:
            #rospy.loginfo("Object not found")
            if self.frame_count >= 30:
                self.end = time.time_ns()
                self.fps = 1000000000 * frame_count / (self.end - self.start)
                self.frame_count = 0
                self.start = time.time_ns()
            if self.fps > 0:
                self.fps_label = "FPS: %.2f" % self.fps
                cv2.putText(self.frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)           
        
            #self.display_objects()

            if self.frame is not None :
                cv2.imshow("Object Detection", self.frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                rospy.loginfo("Bounding box frame is None")
            
            return self.return_value


# def yolo_perception_server() :
#     rospy.init_node("perception_yolo")
#     s = rospy.Service('yolo_service', find_object, find_object_cb)
#     rospy.loginfo("yolo service active")
#     rospy.spin()

# def find_object_cb(req) :

#     rospy.loginfo("Perception request received")
#     wd = WorkpieceDetector(req.object_name)
#     result = wd.control_loop()
#     print("YOLO Result: {}".format(result))

#     return find_objectResponse(wd.return_value)



#print("Total frames: " + str(total_frames))

if __name__ == "__main__" :
    rospy.init_node("perception_yolo")

    wd = WorkpieceDetector()
    rospy.spin()

    # yolo_perception_server()

    # yolo_perception_server()

    
    

