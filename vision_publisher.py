import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import numpy as np
 
class JpegPublisher(Node):

  def __init__(self):

    # Initiate the Node class's constructor and give it a name
    super().__init__('jpeg_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 5 messages.
    self.publisher_ = self.create_publisher(CompressedImage, 'rc_vision', 5)
      
    # We will publish a message every timer_period seconds
    timer_period = 0.5  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    width = 1024
    height = 768
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    self.br = CvBridge()
    self.frame_num = 0
    
  def __del__(self):

    self.cap.release()
   
  def timer_callback(self):

    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()

    self.get_logger().info("cap.read %s" %(str(ret)))
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message		
      t = self.get_clock().now()
      
      
      #msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
      
      msg = CompressedImage()
      msg.format = "jpeg"
      msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

      
      self.get_logger().info('Publishing jpeg frame %d' % (self.frame_num ))
      
      msg.header.frame_id = str(self.frame_num)
      self.frame_num += 1
      msg.header.stamp = t.to_msg()
      self.publisher_.publish(msg)
 
    # Display the message on the console
    self.get_logger().info('frame Published')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = JpegPublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
