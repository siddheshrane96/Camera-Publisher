import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

CAMERA_FREQ = 30 #hz
script_dir = os.path.dirname(os.path.realpath(__file__))
# This won't work because I am running python file directly
# package_share_dir = get_package_share_directory('webcam_publisher')
calib_file = os.path.join(script_dir, '..','calibration', 'calibration_data.npz')

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        # publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos) 
        
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
        # Loading calibration
        self.load_calibration()
    
    def load_calibration(self):
        calib_data = np.load(calib_file)
        self.camera_matrix = calib_data['camera_matrix']
        self.dist_coeff = calib_data['dist_coeff'].flatten()
        self.distortion_model = 'plumb_bob'
        self.R = np.eye(3)
        self.P = np.zeros((3,4))
        self.P[:3,:3] = self.camera_matrix
        print(f"Loaded K=\n{self.camera_matrix} {type(self.camera_matrix)}")
        print(f"Loaded D=\n{self.dist_coeff} {type(self.dist_coeff)}")
    
    def timer_callback(self):
        # self.get_logger().debug("In Timer Callack Image")
        ret, frame = self.cap.read()
        timestamp = self.get_clock().now().to_msg()
        if not ret:
            self.get_logger().error("Failed to capture webcam feed.")
            return
        
        height, width = frame.shape[:2]
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.frame_id
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='passthrough')
        img_msg.header = header

        cam_info_msg = self.make_camera_info_msg(timestamp, height, width)
        self.image_pub.publish(img_msg)
        self.cam_info_pub.publish(cam_info_msg)
        # self.get_logimage_info_pub = ger().debug("Published Image")

    def make_camera_info_msg(self, timestamp, height, width):
        cam_info = CameraInfo()
        cam_info.header.stamp = timestamp
        cam_info.header.frame_id = self.frame_id
        cam_info.height = height
        cam_info.width = width
        cam_info.distortion_model = self.distortion_model
        cam_info.d = self.dist_coeff.tolist()
        cam_info.k = self.camera_matrix.flatten().tolist()
        cam_info.r = self.R.flatten().tolist()
        cam_info.p = self.P.flatten().tolist()
        return cam_info

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