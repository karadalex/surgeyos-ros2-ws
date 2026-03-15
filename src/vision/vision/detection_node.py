import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision.detection_helpers import (
    compute_arm_pose,
    detect_holes,
    detect_mounting_dock,
    detect_robotic_arm,
    detect_white_model,
)
from vision.utils import *


blueColor = (255,0,0)
greenColor = (0,255,0)
redColor = (0,0,255)


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection')

        self.declare_parameter('input_topic', '/camera0/image_raw')
        self.declare_parameter('output_topic', '/camera0/image/processed')
        self.declare_parameter('tf_parent_frame', 'camera_link')
        self.declare_parameter('tf_child_frame', 'vision_target')
        self.declare_parameter('min_hole_area_px', 60.0)
        self.declare_parameter('min_white_model_area_px', 150.0)
        self.declare_parameter('meters_per_pixel', 0.001)
        self.declare_parameter('use_visual_servoing', False)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.tf_parent_frame = self.get_parameter('tf_parent_frame').get_parameter_value().string_value
        self.tf_child_frame = self.get_parameter('tf_child_frame').get_parameter_value().string_value
        self.use_visual_servoing = self.get_parameter('use_visual_servoing').get_parameter_value().bool_value

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.on_image,
            10
        )

        self.pub = self.create_publisher(Image, output_topic, 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")
        self.get_logger().info(
            f"Publishing vision offset TF on /tf: {self.tf_parent_frame} -> {self.tf_child_frame}"
        )

    def on_image(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV image (BGR8)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            out = frame.copy()

            ####################################################################
            # Find the green mounting dock
            ####################################################################
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask, obj, bbox = detect_mounting_dock(hsv)
            if obj is None:
                out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                out_msg.header = msg.header
                self.pub.publish(out_msg)
                return

            cv2.drawContours(out, [obj], -1, (0, 255, 0), 2)

            obj_mask = np.zeros(green_mask.shape, dtype=np.uint8)
            cv2.drawContours(obj_mask, [obj], -1, 255, thickness=cv2.FILLED)

            ####################################################################
            # Find the black robotic arm
            ####################################################################
            black_contours, max_black_contour = detect_robotic_arm(hsv, out.shape)
            if max_black_contour is None:
                out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                out_msg.header = msg.header
                self.pub.publish(out_msg)
                return

            for contour in black_contours:
                cv2.drawContours(out, [contour], -1, (255, 0, 255), 2)

            ####################################################################
            # Find the robotic arm center of mass and pose and move towards gripper
            ####################################################################
            arm_pose = compute_arm_pose(max_black_contour, out.shape)
            if arm_pose is None:
                out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                out_msg.header = msg.header
                self.pub.publish(out_msg)
                return

            new_center = arm_pose['center']
            cv2.arrowedLine(out, new_center, arm_pose['axis_a'], redColor, 2, 8, 0, 0.1)
            cv2.arrowedLine(out, new_center, arm_pose['axis_b'], greenColor, 2, 8, 0, 0.1)
            cv2.circle(out, new_center, 5, (255, 255, 0), -1)

            x, y, w, h = bbox

            ####################################################################
            # Find a white 3d-printed aortic root model (if present) and calculate its center of mass and distance to arm center
            ####################################################################
            min_white_model_area = self.get_parameter(
                'min_white_model_area_px'
            ).get_parameter_value().double_value
            white_model = detect_white_model(
                hsv, bbox, frame.shape, new_center, min_white_model_area
            )
            if white_model is not None:
                cv2.drawContours(out, [white_model['contour']], -1, (255, 255, 255), 2)
                cv2.circle(out, white_model['center'], 5, (0, 165, 255), -1)
                cv2.line(out, new_center, white_model['center'], (0, 165, 255), 2)
                cv2.putText(
                    out,
                    f"root d: {white_model['distance']:.1f}px",
                    (
                        max(0, white_model['center'][0] - 60),
                        max(20, white_model['center'][1] - 10),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )


            ####################################################################
            # Find the mounting dock holes
            ####################################################################
            min_hole_area = self.get_parameter(
                'min_hole_area_px'
            ).get_parameter_value().double_value
            _, hole_contours, hole_target = detect_holes(
                green_mask, obj_mask, new_center, min_hole_area
            )
            for hole in hole_contours:
                (cx, cy), radius = cv2.minEnclosingCircle(hole)
                center = (int(cx), int(cy))
                cv2.circle(out, center, int(radius), (0, 0, 255), 2)
                cv2.circle(out, center, 3, (255, 0, 0), -1)

            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                out,
                f'holes: {len(hole_contours)}',
                (x, max(0, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if self.use_visual_servoing and hole_target is not None:
                # Publish XY correction as a TF translation (camera frame -> vision target).
                # Visual servoing controller will move the arm to reduce the offset to zero.
                meters_per_pixel = self.get_parameter(
                    'meters_per_pixel'
                ).get_parameter_value().double_value
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = self.tf_parent_frame
                t.child_frame_id = self.tf_child_frame
                t.transform.translation.x = float(-hole_target['dx'] * meters_per_pixel)
                t.transform.translation.y = float(-hole_target['dy'] * meters_per_pixel)
                t.transform.translation.z = float(hole_target['distance'] * meters_per_pixel)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_pub.publish(TFMessage(transforms=[t]))

            ####################################################################
            # Publish image feed and calculated center of mass
            ####################################################################

            # Convert OpenCV -> ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
            out_msg.header = msg.header  # preserve timestamp/frame_id
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")


def main():
    rclpy.init()
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
