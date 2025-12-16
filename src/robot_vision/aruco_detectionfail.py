import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String,Bool
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
class uruco(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        self.sub1 = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        self.sub2 = self.create_subscription(String, '/camera1/image_info', self.camera_intrinsics, 10) 
        self.create_subscription(Bool, "/run_aruco_detection", self.start_callback, 10)
        self.pub = self.create_publisher(String, '/robot_report', 10)   
        self.aruco_pub = self.create_publisher(String, '/aruco_report', 10)# this should publish the number of makrers detected so far
        self.markerLength = 0.2 
        self.K = None
        self.distanceCoefficients = np.zeros((5,1))
        self.number_of_markers = 0
        self.run = False #decides whether detection runs or not

    def camera_intrinsics(self, intrinsics_msg):
        intrinsics = intrinsics_msg.data.split(',')
        fx = float(intrinsics[0])
        fy = float(intrinsics[1])
        cx = float(intrinsics[2])
        cy = float(intrinsics[3])
        self.K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]])
        self.distanceCoefficients = np.zeros((5,1)) 
    def run_callback(self, msg):
        if msg.data:
            self.get_logger().info("Aruco detection started")
            self.run=True
        else:
            self.get_logger().info("Aruco detection stopped")

    def image_callback(self, msg):
        if not self.run:
            return
        self.get_logger().info("Received an image!")
        if self.K is None:
            self.get_logger().warn("Camera intrinsics not received yet")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(frame)
        
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.markerLength, self.K, self.distanceCoefficients)
            x = tvecs[0][0][0]
            y = tvecs[0][0][1]
            z = tvecs[0][0][2]
            
            aruco.drawDetectedMarkers(frame, corners, ids)
            print("Detected IDs:", ids)
            local_point = PointStamped()
            local_point.header.frame_id = 'base_link'
            local_point.header.stamp = rclpy.time.Time().to_msg()
            local_point.point.x = x
            local_point.point.y = y
            local_point.point.z = z
            global_point = self.tf_buffer.transform(local_point, 'odom', timeout=rclpy.duration.Duration(seconds=1.0))
            report = "aruco "+ str(ids.flatten().tolist())+ " in position x: "+ str(global_point.point.x)+" y: "+ str(global_point.point.y)+ " z: "+ str(global_point.point.z)
            msg_str = String()
            msg_str.data = report
            self.pub.publish(msg_str)
            self.number_of_markers += len(ids)
            self.aruco_pub.publish(String(data=str(self.number_of_markers)))

def main(args=None):
    rclpy.init(args=args)
    node = uruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    