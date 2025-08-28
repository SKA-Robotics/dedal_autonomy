import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('oak_d_camera_node')

        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        # Ustawienie pipeline i parametrow kamery rgb 
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")

        cam_rgb.video.link(xout_rgb.input)
        # 30 klatek na sekunde timer 1/30
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        try:
            self.device = dai.Device(self.pipeline)
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        except Exception as e:
            self.get_logger().error(f"Nie udało się połączyć {e}")
            rclpy.shutdown()


    def capture_and_publish(self):
        in_rgb = self.q_rgb.tryGet()

        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

            # Konwertuje klatkę na w ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')

            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "oak_d_camera_frame"
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    oak_d_node = ImagePublisher()

    try:
        rclpy.spin(oak_d_node)
    except KeyboardInterrupt:
        pass
    finally:
        oak_d_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()