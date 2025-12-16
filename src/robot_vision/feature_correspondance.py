import cv2
import rclpy

from rclpy.node import Node
from robot_vision.aruco_detection import uruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool

class FeatureCorrespondence(Node):
    def __init__ (self):
        super(). __init__('feature_correspondence_node')
        self.bridge=CvBridge()
        self. camera_sub1=self.create_subscription(Image,'image1',self.image1_callback,10  )
        self. camera_sub2=self.create_subscription(Image,'image2',self.image2_callback,10  )
        self.create_subscription(Bool, "/run_feature_correspondence", self.run_callback, 10)
        self.state_message=self.create_publisher(String, '/robot_report')
        self.run=False
        self.img1=None
        self.img2=None
    def image1_callback(self,msg):
            self.img1=self.bridge.imgmsg_to_cv2(msg,"bgr8")

    def image2_callback(self,msg):
            self.img2=self.bridge.imgmsg_to_cv2(msg,"bgr8")
        
    def run_callback(self,msg):
            if msg.data:
                self.run=True
                self.get_logger().info("Feature correspondence started")
        
    def process_images(self):
            if not self.run:
                return
            if self.img1 is None or self.img2 is None:
                self.get_logger().warn("Images not received yet")
                return
            cv2.imshow('Eiffel 1', self.img1)
            cv2.waitKey(0)

            cv2.imshow('Eiffel 2', self.img2)
            cv2.waitKey(0)

            surf = cv2.xfeatures2d.SURF_create(400)   # Hessian threshold
            kp1, des1 = surf.detectAndCompute(self.img1, None)
            kp2, des2 = surf.detectAndCompute(self.img2, None)

            # Return Values:
            # keypoints: A list or vector of cv2.KeyPoint objects, each representing a 
            #            detected keypoint with properties like its coordinates, size, 
            #            orientation, and response.
            # descriptors: An OutputArray (e.g., NumPy array in Python) containing the 
            #              computed descriptors for each keypoint. The format and size of 
            #              the descriptors depend on the specific feature algorithm used.

            bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
            matches = bf.knnMatch(des1, des2, k=2)

            # Lowe's ratio test
            good = [m for m,n in matches if m.distance < 0.7 * n.distance]
            #good = [m for m,n in matches if m.distance < 0.2 * n.distance]
            for match in good:
                msg_str = String()
                point_in_image1=kp1[match.queryIdx].pt
                point_in_image2=kp2[match.trainIdx].pt
                msg_str.data = f"Movement at x: {point_in_image1[0]}, y: {point_in_image1[1]}"
                self.state_message.publish(msg_str)
            img_matches = cv2.drawMatches(self.img1, kp1, self.img2, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow('SURF matches', img_matches)
            cv2.waitKey(0)
def main(args=None):
            rclpy.init(args=args)
            node = FeatureCorrespondence()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()


'''
old code from ecampus.
img1 = cv2.imread('eiffel1.jpg', 0)   # left image
img2 = cv2.imread('eiffel2.jpg', 0)   # right image

cv2.imshow('Eiffel 1', img1)
cv2.waitKey(0)

cv2.imshow('Eiffel 2', img2)
cv2.waitKey(0)

surf = cv2.xfeatures2d.SURF_create(400)   # Hessian threshold
kp1, des1 = surf.detectAndCompute(img1, None)
kp2, des2 = surf.detectAndCompute(img2, None)

# Return Values:
# keypoints: A list or vector of cv2.KeyPoint objects, each representing a 
#            detected keypoint with properties like its coordinates, size, 
#            orientation, and response.
# descriptors: An OutputArray (e.g., NumPy array in Python) containing the 
#              computed descriptors for each keypoint. The format and size of 
#              the descriptors depend on the specific feature algorithm used.

bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
matches = bf.knnMatch(des1, des2, k=2)

# Lowe's ratio test
good = [m for m,n in matches if m.distance < 0.7 * n.distance]
#good = [m for m,n in matches if m.distance < 0.2 * n.distance]

img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.imshow('SURF matches', img_matches)
cv2.waitKey(0)'''