import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
 
class RpiCamAruco(Node):
  def __init__(self):
    super().__init__('rpi_cam_aruco')
    self.subscription = self.create_subscription(
      Image, 
      '/image_raw/decompressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    converted_frame = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
    
    # Display image
    cv2.imshow("camera", converted_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = RpiCamAruco()
  
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()