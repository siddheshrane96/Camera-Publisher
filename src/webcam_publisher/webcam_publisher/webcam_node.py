import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

CAMERA_FREQ = 30 #hz

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw',10)
        self.timer = self.create_timer(1.0/CAMERA_FREQ, self.timer_callback)
        self.bridge = CvBridge()
        self.frame_id = 'camera_link'

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info("Webcam publisher initialized.")
    
    def timer_callback(self):
        # self.get_logger().debug("In Timer Callack Image")
        ret, frame = self.cap.read()
        timestamp = self.get_clock().now().to_msg()
        if not ret:
            self.get_logger().error("Failed to capture webcam feed.")
            return
        
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.frame_id
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='passthrough')
        img_msg.header = header
        self.publisher_.publish(img_msg)
        # self.get_logger().debug("Published Image")

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("Webcam released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    webcam_pub_node = None

    try:
        webcam_pub_node = WebcamPublisher()
        if webcam_pub_node.cap.isOpened():
            rclpy.spin(webcam_pub_node)
    except KeyboardInterrupt:
        if webcam_pub_node is not None:
            webcam_pub_node.get_logger().info("Shutting down...")
    finally:
        if webcam_pub_node is not None:
            webcam_pub_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()