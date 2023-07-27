import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time
import math
 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      

    
    self.publisher_ = self.create_publisher(Image, 'color_camera/image_raw', 10)
    while True:
      self.start = time.time()
      self.image = read_image_bgr('/home/noemoji041/ros2_workspaces/percepros2_ws/src/ros-perception-pipeline/object_detection/object_detection/test2.png')

      self.br = CvBridge()
    

      self.draw = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)


      
      self.end = time.time()
      self.fps = math.ceil(1/(self.end - self.start))
      # self.draw = cv2.putText(self.draw, str(self.fps), (00, 185),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
      self.publisher_.publish(self.br.cv2_to_imgmsg(self.draw, encoding='rgb8'))
      self.get_logger().info('Publishing video frame')
      # cv2.imshow("ok", self.draw)
      # cv2.waitKey(1)

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  # rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()