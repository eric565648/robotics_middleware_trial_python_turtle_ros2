import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError

class WebcamSub(Node):
    def __init__(self):
        super().__init__('stream_node')

        self.bridge = CvBridge()

        # define subscriber
        self.img_subscription = self.create_subscription(Image, 'image_raw', self.img_callback, 5)
        self.img_subscription # prevent unused varaibale warning

    def img_callback(self, img_msg):
        
        # bridging from img msg to cv2 img
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().info(e)
        
        # show image
        cv2.namedWindow("Image")
        if cv_image is not None:
            cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

def main(args=None):

    rclpy.init(args=args)

    imgsub_obj = WebcamSub()
    rclpy.spin(imgsub_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imgsub_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()