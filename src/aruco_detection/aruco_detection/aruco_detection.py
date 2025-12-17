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
from collections import deque

class MarkerTrack:
    def __init__(self, win=25):
        self.samples = deque(maxlen=win)   # each: np.array([x,y,z])
        self.published = False

    def add(self, p_world, gate_dist=0.6):
        """Add a sample if it's not a big outlier vs the running median."""
        p = np.asarray(p_world, dtype=float)

        if len(self.samples) >= 3:
            med = np.median(np.vstack(self.samples), axis=0)
            if np.linalg.norm(p - med) > gate_dist:
                return False  # reject outlier

        self.samples.append(p)
        return True

    def estimate_and_spread(self):
        """Return (robust_estimate, spread_m). Spread is RMS radius from median."""
        pts = np.vstack(self.samples)
        med = np.median(pts, axis=0)
        spread = float(np.sqrt(np.mean(np.sum((pts - med) ** 2, axis=1))))
        return med, spread

    def ready(self, n_min=10, spread_thresh=0.12):
        if len(self.samples) < n_min:
            return False
        _, spread = self.estimate_and_spread()
        return spread < spread_thresh

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
                # Per-marker tracking (publish once when stable)
        self.tracks = {}  # marker_id -> MarkerTrack
        self.declare_parameter('track_window', 25)
        self.declare_parameter('track_min_samples', 10)
        self.declare_parameter('track_spread_thresh', 0.12)  # meters
        self.declare_parameter('track_gate_dist', 0.6)       # meters

        self.track_window = int(self.get_parameter('track_window').value)
        self.track_min_samples = int(self.get_parameter('track_min_samples').value)
        self.track_spread_thresh = float(self.get_parameter('track_spread_thresh').value)
        self.track_gate_dist = float(self.get_parameter('track_gate_dist').value)


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

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)

            # Transform this observation to world
            x, y, z = tvecs[i][0]
            world_coords = self.camera_xyz_to_world(x, y, z, stamp_msg=msg.header.stamp)
            if world_coords is None:
                self.get_logger().warn(f"Could not transform marker ID {marker_id} to world frame.")
                continue

            # Get / create track
            if marker_id not in self.tracks:
                self.tracks[marker_id] = MarkerTrack(win=self.track_window)
                self.get_logger().info(f"Tracking started for marker ID {marker_id}")

            track = self.tracks[marker_id]

            # If already published, do nothing (still draw axes for viz)
            if track.published:
                cv2.drawFrameAxes(frame, self.K, self.dist, rvecs[i], tvecs[i], 0.1)
                continue

            # Add sample (with outlier rejection)
            accepted = track.add(world_coords, gate_dist=self.track_gate_dist)
            if not accepted:
                self.get_logger().debug(f"Rejected outlier for marker {marker_id}: {world_coords}")
                cv2.drawFrameAxes(frame, self.K, self.dist, rvecs[i], tvecs[i], 0.1)
                continue

            # If stable, publish ONCE
            if track.ready(n_min=self.track_min_samples, spread_thresh=self.track_spread_thresh):
                est, spread = track.estimate_and_spread()
                wx, wy, wz = float(est[0]), float(est[1]), float(est[2])

                track.published = True

                self.get_logger().info(
                    f"Marker {marker_id} STABLE (n={len(track.samples)}, spread={spread:.3f}m) "
                    f"-> world=({wx:.3f}, {wy:.3f}, {wz:.3f})"
                )

                # Publish String report once
                report_msg = String()
                report_msg.data = f"aruco {marker_id} in position x: {wx}, y: {wy}, z: {wz}"
                self.report_pub.publish(report_msg)

                # Publish ArucoMarkerArray once (append once)
                marker = ArucoMarker()
                marker.id = marker_id
                marker.pose.header.stamp = msg.header.stamp
                marker.pose.header.frame_id = self.world_frame  # important: world frame now
                marker.pose.pose.position.x = wx
                marker.pose.pose.position.y = wy
                marker.pose.pose.position.z = wz

                self.aruco_msg.markers.append(marker)
                self.pub_markers.publish(self.aruco_msg)

            # Optional: draw axes
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
