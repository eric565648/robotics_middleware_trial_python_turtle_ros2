import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Webcam_Impl(Node): 
    def __init__(self):
        super().__init__('webcam')

        # initialize a publisher
        self.img_publisher = self.create_publisher(Image, 'image_raw', 1)

        #initializa camera parameters
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3,320)
        self.camera.set(4,240)

        self.bridge=CvBridge()

        # create timer
        self.timer = self.create_timer(0.03, self.capture_frame)

    def capture_frame(self):

        rval,img_data = self.camera.read()
        if rval:
            self.img_publisher.publish(self.bridge.cv2_to_imgmsg(img_data, "bgr8"))
            return img_data 
        else:
            print("error")

def main(args=None):

    # initial a ros2
    rclpy.init(args=args)

    # initialize a camera object/ros2 node
    webcam=Webcam_Impl()
    webcam.get_logger().info('Webcam Node Started!') 

    # spin the node (otherwise it only execute once)
    rclpy.spin(webcam)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webcam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    