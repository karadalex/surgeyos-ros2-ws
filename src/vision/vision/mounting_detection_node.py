import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class MountingDetection(Node):
    def __init__(self):
        super().__init__('mounting_detection')

        self.declare_parameter('input_topic', '/image')
        self.declare_parameter('output_topic', '/image/processed')
        self.declare_parameter('use_canny', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.on_image,
            10
        )

        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")

    def on_image(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV image (BGR8)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example processing: grayscale + optional Canny
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            use_canny = self.get_parameter('use_canny').get_parameter_value().bool_value
            if use_canny:
                edges = cv2.Canny(gray, 80, 160)
                out = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            else:
                out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # Convert OpenCV -> ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            out_msg.header = msg.header  # preserve timestamp/frame_id
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")


def main():
    rclpy.init()
    node = MountingDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
