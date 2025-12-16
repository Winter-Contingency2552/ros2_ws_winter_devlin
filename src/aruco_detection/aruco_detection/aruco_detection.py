#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from aruco_interfaces.msg import ArucoMarker, ArucoMarkerArray


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.bridge = CvBridge()

        # Params (optional but nice)
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('camera_info_topic', '/camera1/camera_info')
        self.declare_parameter('marker_length', 0.2)  # meters
        self.declare_parameter('robot_report', '/robot_report')

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_length = float(self.get_parameter('marker_length').value)
        robot_report = self.get_parameter('robot_report').value

        # Subscribers / publisher
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.caminfo_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.report_pub = self.create_publisher(String, robot_report, 10)
        self.pub_markers = self.create_publisher(ArucoMarkerArray, "/aruco/markers", 10)

        # TF2 buffer and listener reading world to base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_frame = "world"
        self.camera_frame = "camera_link"

        self.aruco_msg = ArucoMarkerArray()
        self.aruco_msg.header.stamp = self.get_clock().now().to_msg()
        self.aruco_msg.header.frame_id = self.world_frame

        # Intrinsics storage
        self.K = None
        self.dist = None

        # ArUco setup (OpenCV 4.5+ compatible)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        #self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params = aruco.DetectorParameters()
        self.seen_markers = []

        self.get_logger().info(f"Listening to {image_topic} and {camera_info_topic}")

    def camera_xyz_to_world(self, x_rel, y_rel, z_rel, stamp_msg=None):
        # Map your ArUco axes -> camera_link (+X forward, +Y left, +Z up)
        x_cam = float(z_rel)  # forward
        y_cam = -1 * float(x_rel)  # left
        z_cam = -1 * float(y_rel)  # up

        p_cam = PointStamped()
        p_cam.header.frame_id = self.camera_frame  # "camera_link"
        p_cam.header.stamp = stamp_msg if stamp_msg is not None else self.get_clock().now().to_msg()
        p_cam.point.x, p_cam.point.y, p_cam.point.z = x_cam, y_cam, z_cam

        try:
            t = rclpy.time.Time.from_msg(p_cam.header.stamp)

            ok = self.tf_buffer.can_transform(
                self.world_frame,
                self.camera_frame,
                t,
                timeout=Duration(seconds=0.2),
            )
            if not ok:
                self.get_logger().warn("TF not available at measurement time; falling back to latest TF")
                p_cam.header.stamp = rclpy.time.Time().to_msg()  # latest
                t = rclpy.time.Time()

            p_world = self.tf_buffer.transform(
                p_cam,
                self.world_frame,
                timeout=Duration(seconds=0.2),
            )
            return (p_world.point.x, p_world.point.y, p_world.point.z)

        except (tf2_ros.ExtrapolationException,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

    def camera_info_callback(self, msg: CameraInfo):
        # msg.k is a 3x3 row-major camera matrix
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # msg.d can be length 0, 4, 5, 8, etc. depending on model
        if len(msg.d) > 0:
            self.dist = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            # No distortion provided
            self.dist = np.zeros((5, 1), dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.K is None or self.dist is None:
            self.get_logger().warn("No CameraInfo received yet (K/dist unknown). Skipping frame.")
            return

        # Convert ROS Image -> OpenCV BGR image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Detect markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Compatible approach across OpenCV 4.5.x
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            # non-blocking display if you want it
            cv2.imshow("Aruco Detection", frame)
            cv2.waitKey(1)
            return

        # Estimate pose for each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.K, self.dist
        )

        # Draw + publish results
        aruco.drawDetectedMarkers(frame, corners, ids)

        report_lines = []
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in self.seen_markers:
                self.seen_markers.append(marker_id)
                self.get_logger().info(f"New marker detected: ID {marker_id}")
                x, y, z = tvecs[i][0]  # meters in camera frame
                report_lines.append(f"id={int(marker_id)} xyz=({x:.3f}, {y:.3f}, {z:.3f})")

                # Transform to world frame
                world_coords = self.camera_xyz_to_world(x, y, z, stamp_msg=msg.header.stamp)
                if world_coords is not None:
                    wx, wy, wz = world_coords
                    self.get_logger().info(f"Marker ID {marker_id} in world frame: ({wx:.3f}, {wy:.3f}, {wz:.3f})")
                    report_lines.append(f"{marker_id} in position x: {wx}, y: {wy}, z{wz}")
                    report_msg = String()
                    report_msg.data = "aruco " + "; ".join(report_lines)
                    self.report_pub.publish(report_msg)

                    self.get_logger().info(report_msg.data)

                    # Publish ArucoMarkerArray
                    marker = ArucoMarker()
                    marker.id = int(marker_id)
                    marker.pose.header.stamp = msg.header.stamp
                    marker.pose.header.frame_id = msg.header.frame_id

                    marker.pose.pose.position.x = wx
                    marker.pose.pose.position.y = wy
                    marker.pose.pose.position.z = wz
                    self.aruco_msg.markers.append(marker)
                    self.pub_markers.publish(self.aruco_msg)
                else:
                    self.get_logger().warn(f"Could not transform marker ID {marker_id} to world frame.")

               
            # Optional: draw axes on the marker
            cv2.drawFrameAxes(frame, self.K, self.dist, rvecs[i], tvecs[i], 0.1)

        

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
