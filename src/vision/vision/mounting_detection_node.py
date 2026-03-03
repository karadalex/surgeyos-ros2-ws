import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision.utils import *


blueColor = (255,0,0)
greenColor = (0,255,0)
redColor = (0,0,255)


class MountingDetection(Node):
    def __init__(self):
        super().__init__('mounting_detection')

        self.declare_parameter('input_topic', '/camera0/image_raw')
        self.declare_parameter('output_topic', '/camera0/image/processed')
        self.declare_parameter('tf_parent_frame', 'camera_link')
        self.declare_parameter('tf_child_frame', 'vision_target')
        self.declare_parameter('min_hole_area_px', 60.0)
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
            # Segment green regions in HSV space.
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_green = np.array([35, 50, 40], dtype=np.uint8)
            upper_green = np.array([90, 255, 255], dtype=np.uint8)
            green_mask = cv2.inRange(hsv, lower_green, upper_green)

            kernel = np.ones((5, 5), np.uint8)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            object_contours, _ = cv2.findContours(
                green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not object_contours:
                out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
                out_msg.header = msg.header
                self.pub.publish(out_msg)
                return

            # Use the largest green connected region as the target object.
            obj = max(object_contours, key=cv2.contourArea)
            object_area = cv2.contourArea(obj)
            if object_area < 500.0:
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
            # Detect black object (assuming roarm robotic arm)
            lower_black = np.array([0, 0, 0], dtype=np.uint8)
            upper_black = np.array([200, 255, 100], dtype=np.uint8)  # tune V upper bound
            black_mask = cv2.inRange(hsv, lower_black, upper_black)

            kernel = np.ones((2, 2), np.uint8)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            black_contours, _ = cv2.findContours(
                black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            mask = np.zeros(out.shape[:2], dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10,10), np.uint8))
            cv2.drawContours(mask, black_contours, -1, 255, thickness=cv2.FILLED)  # union fill
            union_black_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # union_contours now are merged outer contours

            max_black_contour_area = 0
            max_black_contour = None
            for c in union_black_contours:
                if cv2.contourArea(c) < 300:   # tune area threshold
                    continue
                if cv2.contourArea(c) > max_black_contour_area:
                    max_black_contour_area = cv2.contourArea(c)
                    max_black_contour = c
                cv2.drawContours(out, [c], -1, (255, 0, 255), 2)  # magenta outline

            ####################################################################
            # Find the robotic arm center of mass and pose and move towards gripper
            ####################################################################
            black_pts, black_mask = points_inside_contour(max_black_contour, out.shape[:2])
            cm = np.mean(black_pts, axis=0).astype(int)

            # Correct the orientation vector direction to point towards the gripper (assuming gripper is on the right side of the arm)
            # contour: Nx1x2
            ellipse = cv2.fitEllipse(max_black_contour)
            (cx, cy), (w, h), angle_deg = ellipse   # center, axes, rotation

            # major-axis direction
            theta = np.deg2rad(angle_deg)
            ux, uy = np.cos(theta), np.sin(theta)

            # if OpenCV returns swapped axes, rotate 90 deg
            if h > w:
                ux, uy = -np.sin(theta), np.cos(theta)

            # shift center toward one side (e.g. +major direction)
            shift = 0.45 * max(w, h)   # 25% of major axis length (tune)
            new_cx = cx - shift * ux
            new_cy = cy - shift * uy

            # If point at dome distance next to the center is outside the contour then this is the correct direction, otherwise flip
            maybe_inside = cv2.pointPolygonTest(max_black_contour, (new_cx-2*shift*ux, new_cy-2*shift*uy), False) >= 0
            if maybe_inside:
                new_cx = cx + shift * ux
                new_cy = cy + shift * uy

            new_center = (int(new_cx), int(new_cy))

            a, b = orientationVectors(black_pts)
            a = new_center + a
            b = new_center + b
            cv2.arrowedLine(out, (new_center[0],new_center[1]), (a[0], a[1]), redColor, 2, 8, 0, 0.1)
            cv2.arrowedLine(out, (new_center[0],new_center[1]), (b[0], b[1]), greenColor, 2, 8, 0, 0.1)
            cv2.circle(out, tuple(new_center), 5, (255, 255, 0), -1)  # magenta center of mass

            ####################################################################
            # Find the mounting dock holes
            ####################################################################
            # Holes appear as non-green islands inside the green object.
            holes_mask = cv2.bitwise_and(cv2.bitwise_not(green_mask), obj_mask)
            holes_mask = cv2.morphologyEx(holes_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

            hole_contours, _ = cv2.findContours(
                holes_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            min_hole_area = self.get_parameter(
                'min_hole_area_px'
            ).get_parameter_value().double_value
            hole_count = 0
            for hole in hole_contours:
                area = cv2.contourArea(hole)
                if area < min_hole_area:
                    continue
                hole_count += 1
                (cx, cy), radius = cv2.minEnclosingCircle(hole)
                center = (int(cx), int(cy))
                cv2.circle(out, center, int(radius), (0, 0, 255), 2)
                cv2.circle(out, center, 3, (255, 0, 0), -1)

            x, y, w, h = cv2.boundingRect(obj)
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                out,
                f'holes: {hole_count}',
                (x, max(0, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            # Calculate distance from arm center to hole center in pixel space
            max_hole_contour_area = 0
            max_hole_contour = None
            for c in hole_contours:
                if cv2.contourArea(c) > max_hole_contour_area:
                    max_hole_contour_area = cv2.contourArea(c)
                    max_hole_contour = c
            hole_cm = np.mean(max_hole_contour.reshape(-1, 2), axis=0).astype(int)
            dx = new_cx - hole_cm[0]
            dy = new_cy - hole_cm[1]
            distance = np.sqrt(dx**2 + dy**2)


            if self.use_visual_servoing:
                # Publish XY correction as a TF translation (camera frame -> vision target).
                # Visual servoing controller will move the arm to reduce the offset to zero.
                meters_per_pixel = self.get_parameter(
                    'meters_per_pixel'
                ).get_parameter_value().double_value
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = self.tf_parent_frame
                t.child_frame_id = self.tf_child_frame
                t.transform.translation.x = float(-dx * meters_per_pixel)
                t.transform.translation.y = float(-dy * meters_per_pixel)
                t.transform.translation.z = float(distance * meters_per_pixel)
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
