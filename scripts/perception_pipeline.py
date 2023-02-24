#!/usr/bin/env python

import rospy
import time 
import tf
import Utils
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseStamped
from object_msgs.msg import ObjectPose
from sensor_msgs.msg import Image
from find_object_2d.msg import ObjectsStamped, DetectionInfo

from actionlib import SimpleActionServer
from ebot_handler.msg import PerceptionAction, PerceptionResult, PerceptionFeedback
from rospy.exceptions import ROSException

import roslaunch

import message_filters


class ObjectPerception:
    '''A class used to subscribe to get object related data from find_object_2d and tf and return it'''

    def __init__(self):
        # subscriber initialization to subscribe to /objectStamped topic
        self.ob_sub = message_filters.Subscriber('/info', DetectionInfo)
        self.image_sub = message_filters.Subscriber('camera/color/image_raw2', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.ob_sub, self.image_sub], 10, 0.1, allow_headerless=False)
        self.bridge = CvBridge()
        self.ts.registerCallback(self.perception_cb)

        # listener initialization used to lookup required transforms 
        self.tf_listener = tf.TransformListener()

    def perception_cb(self, detection_info, ros_img):
        '''callback function for ob_subscriber'''

        ids = detection_info.ids
        widths = detection_info.widths
        heights = detection_info.heights
        filePaths = detection_info.filePaths
        inliers = detection_info.inliers
        outliers = detection_info.outliers
        homographies = detection_info.homographies

        ob_data = zip(ids, widths, heights, filePaths, inliers, outliers, homographies)

        self.ob_data = Utils.dict_from_DetectionInfo(ob_data)

        self.ros_img = ros_img

        # unregister subscriber after recieving required data
        self.ob_sub.unregister()
        self.image_sub.unregister()
        
    def get_ob_data(self):
        '''function used to return detected object name and transforms w.r.t base_link
            parameters: No parameters
            returns: list of dictionary with detected object name and transform'''

        if not self.ob_data:
            return None 

        else:
            for ob in self.ob_data:

               # get detected object name using ob_id
               ob['name'] = Utils.get_ob_name(ob['filePath'])

               # wait for required to transform to become available
               self.tf_listener.waitForTransform('base_link', 'object_' + str(ob['id']), rospy.Time(0), rospy.Duration(3))
               # lookup the required transform between object and base_link
               (ob['trans'], ob['rot']) = self.tf_listener.lookupTransform('base_link', 'object_' + str(ob['id']), rospy.Time(0))

            return self.ob_data 

    def display_objects(self):
        '''function to create cv_bridge object and use to it create bounding 
            box and write object name on the received camera image'''

        # create opencv image from image message
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(self.ros_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        for ob in self.ob_data:
            h = ob['height'] - 1 
            w = ob['width'] - 1
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            print(pts)

            # create homography matrix
            homography_matrix = np.float32([[ob['homography'][0], ob['homography'][3], ob['homography'][6]], [ob['homography'][1], ob['homography'][4], ob['homography'][7]], [ob['homography'][2], ob['homography'][5], ob['homography'][8]]])
            box = cv2.perspectiveTransform(pts, homography_matrix) 

            # create bounding box on the image
            self.cv_image = cv2.polylines(self.cv_image, [np.int32(box)], True, (255, 0, 0), 3)

            # write object name
            self.cv_image = cv2.putText(self.cv_image, ob['name'], tuple(box[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("Detected Objects", self.cv_image)
        k = cv2.waitKey(0)

        # wait for ESC key to exit
        if k == 27:
            cv2.destroyAllWindows()


class PerceptionActionServer: 
    '''class to generate ObjectPose message and publish it on detection_info topic'''

    def __init__(self, name): 
        self.server_name = name
        self.perception_server = SimpleActionServer(self.server_name, PerceptionAction, self.execute_cb, auto_start=False)
        self._feedback = PerceptionFeedback()
        self._result = PerceptionResult()

        rospy.loginfo("Starting Perception Action Server")
        self.perception_server.start()
        rospy.loginfo("Perception Action Server STARTED")
        
    def execute_cb(self, goal): 
        '''Execute callback function called when goal received by Perception Action Server
            parameters: goal->PerceptionAction->goal sent by Perception Action Client
            returns: nothing'''

        rospy.loginfo("GOAL recieved")

        # wait for find_object_2d to start publishing on /info topic
        # Send false goal if timeout is reached
        try:
            rospy.wait_for_message('/info', DetectionInfo, rospy.Duration(3))
        except ROSException:
            rospy.logerr("WAITING FOR /info MESSAGE TIMEOUT!")
            rospy.logwarn("Maybe find_object_2d has not started fully")
            rospy.logwarn("SENDING FALSE SUCCESS TO CLIENT")
            self._result.ob_success = False
            self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return

        # initialize ObjectPerception object if message is received on /info to initialize image_sub and ob_sub
        self.ob_perception = ObjectPerception()

        # wait 3 seconds to allow subscriber to make connections
        rospy.sleep(3)

        # get object data published from find_object_2d 
        ob_data = self.ob_perception.get_ob_data()

        # if no object is detected return False success to client
        if ob_data is None:
            rospy.logerr("NO OBJECT DETECTED BY FIND_OBJECT_2D")
            self._result.ob_success = False
            self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return

        # display image with bounding box on objects
        self.ob_perception.display_objects()

        ob_detected = 0 

        # get the pose of the object to be picked
        for ob in ob_data:
            rospy.loginfo("[RESULT]: {} IDENTIFIED".format(ob['name'].upper()))
            if ob['name'] == goal.ob_name:
                self.msg = ObjectPose()
                self.msg.name = ob['name'] 
                self.msg.pose = PoseStamped()
                self.msg.pose.pose.position.x = ob['trans'][0] 
                self.msg.pose.pose.position.y = ob['trans'][1]
                self.msg.pose.pose.position.z = ob['trans'][2]
                self.msg.pose.pose.orientation.x = ob['rot'][0] 
                self.msg.pose.pose.orientation.y = ob['rot'][1]
                self.msg.pose.pose.orientation.z = ob['rot'][2]
                self.msg.pose.pose.orientation.w = ob['rot'][3]

                ob_detected = 1 

            else:
                self._feedback.ob_detected = ob['name']
                self.perception_server.publish_feedback(self._feedback)

        # return True success and ObjectPose message of the object to be picked if it was detected
        # else return false success and empty pose
        if ob_detected:
            self._result.ob_success = True
            self._result.ob_data = self.msg
            self.perception_server.set_succeeded(self._result)
            return

        else:
            self._result.ob_success = False
            self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return


if __name__ == "__main__":
    try:
        rospy.init_node('perception_pipeline')

        # initialize perception action server
        perception_action_server = PerceptionActionServer('/Perception_action_server')

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

