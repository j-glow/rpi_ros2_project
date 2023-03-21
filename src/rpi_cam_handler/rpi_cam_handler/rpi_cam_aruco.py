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
    

    #find aruco tags
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    
    markerCorners, markerIds, _ = detector.detectMarkers(converted_frame)

    #draw found markers
    arucoImg = converted_frame
    if len(markerCorners)>0:
      markerIds.flatten()

      for (corner, ID) in zip(markerCorners, markerIds):
        markerCorners = corner.reshape((4,2))
        (topLeft, topRight, bottomRight, bottomLeft) = markerCorners

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        #draw frame
        cv2.line(arucoImg, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(arucoImg, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(arucoImg, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(arucoImg, bottomLeft, topLeft, (0, 255, 0), 2)

        #draw center point
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(arucoImg, (cX, cY), 4, (0, 0, 255), -1)

        #draw ID
        cv2.putText(arucoImg, str(ID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow("markers found", arucoImg)

    if len(markerCorners)>0:
      if 0 in markerIds:
        converted_frame = cv2.flip(converted_frame, 0)
      if 1 in markerIds:
        converted_frame = cv2.flip(converted_frame, 1)

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