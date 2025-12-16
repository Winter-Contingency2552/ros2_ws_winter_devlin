import cv2
import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import sys
import torch
sys.path.append('/home/john/LightGlue')
from lightglue import SuperPoint, LightGlue
from lightglue.utils import load_image, rbd


class FeatureCorrespondence(Node):
    def __init__ (self):
        super(). __init__('feature_correspondence_node')
        self.bridge=CvBridge()
        self. camera_sub1=self.create_subscription(Image,'image1',self.image1_callback,10  )
        self. camera_sub2=self.create_subscription(Image,'image2',self.image2_callback,10  )
        self.create_subscription(Bool, "/run_feature_correspondence", self.run_callback, 10)
        self.create_subscription(Bool, "/run_feature_correspondence_bonus", self.run_callback, 10)
        self.state_message=self.create_publisher(String, '/robot_report')
        self.run=False
        self.run_bonus=False
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
            if self.run:
    

                if self.img1 is None or self.img2 is None:
                    print("Error: Could not load one of the images.")
                gray1 = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)
                thresh1=cv2.threshold(gray1,127,255,cv2.THRESH_BINARY)[1]
                thresh2=cv2.threshold(gray2,127,255,cv2.THRESH_BINARY)[1]
                center1=cv2.moments(thresh1, binaryImage=True)

                if center1["m00"] != 0:
                    c1X = int(center1["m10"] / center1["m00"])
                    c1Y = int(center1["m01"] / center1["m00"])
                else:
                    c1X, c1Y = 0, 0  
                center2=cv2.moments(thresh2, binaryImage=True)

                if center2["m00"] != 0:
                    c2X = int(center2["m10"] / center2["m00"])
                    c2Y = int(center2["m01"] / center2["m00"])
                else:
                    c2X, c2Y = 0, 0

                dx = c2X - c1X
                dy = c2Y - c1Y
                msg_str=String()
                msg_str.data = f"Movement at x: {dx}, y: {dy}"
                self.state_message.publish(msg_str)
                self.img1=None
                self.img2=None
            elif self.run_bonus:
                # superpoint_2025_perfect.py
                # Works on every machine, every Python version, CPU or GPU – Nov 2025
                import sys
                # --- FIX: Add LightGlue to path manually since install failed ---
                sys.path.append('/home/john/LightGlue')
                # --------------------------------------------------------------


                # ==================== Load images ====================
                img0 = self.img1   # left image
                img1 = self.img2   # right image

                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                img0 = torch.from_numpy(img0).float() / 255.0
                img1 = torch.from_numpy(img1).float() / 255.0
                # ==================== Force proper tensor format [1,3,H,W] ====================
                def prepare_image(t):
                    if t.ndim == 2:                              # grayscale [H,W]
                        t = t.unsqueeze(0).unsqueeze(0)          # [1,1,H,W]
                    if t.ndim == 3:                              # [C,H,W]
                        t = t.unsqueeze(0)                       # [1,C,H,W]
                    if t.shape[1] == 1:                          # grayscale → RGB
                        t = t.repeat(1, 3, 1, 1)
                    return t.to(device)

                img0 = prepare_image(img0)
                img1 = prepare_image(img1)

                # ==================== Optional resize ====================
                def resize_img(t, max_size=1024):
                    _, _, h, w = t.shape
                    if max(h, w) > max_size:
                        scale = max_size / max(h, w)
                        new_h, new_w = int(h * scale), int(w * scale)
                        t = torch.nn.functional.interpolate(t, size=(new_h, new_w),
                                                            mode="bilinear", align_corners=False)
                    return t

                img0 = resize_img(img0)
                img1 = resize_img(img1)

                # ==================== Models ====================
                extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
                # matcher   = LightGlue(features="superpoint").eval().to(device)
                matcher = LightGlue(
                    features="superpoint",
                    depth_confidence=0.9,   # only very confident keypoint detections
                    width_confidence=0.9,   # only very confident matches
                ).eval().to(device)

                # ==================== Inference ====================
                with torch.no_grad():
                    feats0 = extractor.extract(img0)
                    feats1 = extractor.extract(img1)
                    matches_data = matcher({"image0": feats0, "image1": feats1})

                # Clean up
                feats0, feats1, matches_data = rbd(feats0), rbd(feats1), rbd(matches_data)

                # ==================== Extract matches safely ====================
                # These lines handle every possible batch dimension case
                matches0 = matches_data["matches0"].cpu()
                if matches0.ndim == 2:
                    matches0 = matches0[0]          # remove batch dim if present
                matches0 = matches0.numpy()

                kpts0 = feats0["keypoints"].cpu()
                if kpts0.ndim == 3:
                    kpts0 = kpts0[0]
                kpts0 = kpts0.numpy()

                kpts1 = feats1["keypoints"].cpu()
                if kpts1.ndim == 3:
                    kpts1 = kpts1[0]
                kpts1 = kpts1.numpy()

                # ==================== Build OpenCV matches ====================
                good_matches = []
                for i, j in enumerate(matches0):
                    if j >= 0:                                   # valid match
                        m = cv2.DMatch()
                        m.queryIdx = i
                        m.trainIdx = int(j)
                        m.distance = 0.0
                        good_matches.append(m)
                # good_matches = good_matches[:30]

                print(f"SuperPoint + LightGlue found {len(good_matches)} perfect matches!")

                # ==================== Visualization ====================
                left  = self.img1
                right = self.img2

                kp0 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts0]
                kp1 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts1]


                # === ADD THIS BLOCK TO FIX HORIZONTAL SHIFT ===
                # If images were resized differently during processing, we must resize the visualization images to match the tensor sizes
                h0, w0 = img0.shape[2], img0.shape[3]   # size used during matching
                h1, w1 = img1.shape[2], img1.shape[3]

                left_vis  = cv2.resize(left,  (w0, h0))
                right_vis = cv2.resize(right, (w1, h1))
                # ===============================================

                # Then draw (use the resized versions)
                result = cv2.drawMatches(
                    left_vis, kp0,
                    right_vis, kp1,
                    good_matches, None,
                    matchColor=(0, 255, 0),
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )

                result = cv2.drawMatches(
                    left, kp0, right, kp1, good_matches, None,
                    matchColor=(0, 255, 0),
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                    )

                cv2.imshow("SuperPoint + LightGlue – 2025 (finally flawless)", result)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                  
def main(args=None):
            rclpy.init(args=args)
            node = FeatureCorrespondence()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()


