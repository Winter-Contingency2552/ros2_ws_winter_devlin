import rclpy 
from std_msgs.msg import Image, String
import cv2
import cv2.aruco as aruco
import numpy as np
class uruco:
    def __init__(self):
        super().__init__('aruco_detector_node', anonymous=True)
        self.sub1=self.create_subscription('/camera1/image_raw', Image, self.image_callback)
        self.sub2=self.create_subscription('/camera1/image_info', String, self.camera_intrinsics) 
        self.pub=self.create_publisher('/robot_report', String, queue_size=10)   
        self.markerLength = 0.2 
    def camera_intrinsics(self, intrinsics_msg):
        intrinsics = intrinsics_msg.data.split(',')
        fx = float(intrinsics[0])
        fy = float(intrinsics[1])
        cx = float(intrinsics[2])
        cy = float(intrinsics[3])
        global K
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]])
        global distanceCoefficients
        distanceCoefficients = np.zeros((5,1)) 


    def image_callback(self, msg):
        rclpy.loginfo("Received an image!")
        frame = cv2.imread(msg.data)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(frame)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers( corners, self.markerLength,K,distanceCoefficients)
        x = tvecs[0][0][0]
        y = tvecs[0][0][1]
        z = tvecs[0][0][2]
        # Draw results
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            print("Detected IDs:", ids)
            self.pub.publish("aruco "+ str(ids.flatten().tolist())+ "in postion x: "+ x+" y: "+ y+ " z: "+ z)
            cv2.imshow("Detected markers", frame)
            cv2.waitKey(0)

    def run(self):
        rclpy.spin()
if __name__ == '__main__':
    uruco.run()
    