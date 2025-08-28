import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import TagLocation
from rclpy.parameter import ParameterDescriptor, ParameterType


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        # Marker size w Metrach !!!
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_150')
        self.declare_parameter('camera_matrix', [0.0]*9)
        self.declare_parameter('distortion_coeffs', [0.0]*5)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.location_publisher = self.create_publisher(TagLocation, 'tag_location', 10)

        dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.__getattribute__(dict_name))
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        
    
    def image_callback(self, msg):
        marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        camera_matrix = np.array(self.get_parameter('camera_matrix').get_parameter_value().double_array).reshape(3, 3)
        distortion_coeffs = np.array(self.get_parameter('distortion_coeffs').get_parameter_value().double_array)
        
        half_length = marker_size / 2
        obj_points = np.array([
            [-half_length,  half_length, 0],
            [ half_length,  half_length, 0],
            [ half_length, -half_length, 0],
            [-half_length, -half_length, 0]
        ], dtype=np.float32)

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        corners, ids, rejected = self.detector.detectMarkers(cv_image)
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                # ERC 2025 lotnisko na znaczniku 102
                if marker_id[0] == 102:
                    img_points = corners[i]
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, camera_matrix, distortion_coeffs
                    )
                    
                    if success:
                        location_msg = TagLocation()
                        location_msg.x_distance = tvec[0][0]
                        location_msg.y_distance = tvec[1][0]
                        location_msg.z_distance = tvec[2][0]
                        
                        self.location_publisher.publish(location_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()