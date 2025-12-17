import cv2
import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import sys
import torch
sys.path.append('/home/jeff/LightGlue')
from lightglue import SuperPoint, LightGlue
from lightglue.utils import load_image, rbd


class FeatureCorrespondence(Node):
    def __init__ (self):
        super(). __init__('feature_correspondance_node')
        self.get_logger().info('Initializing Feature Correspondence Node...')
        self.bridge=CvBridge()
        self.camera_sub1=self.create_subscription(Image,'/image1',self.image1_callback,10)
        self.camera_sub2=self.create_subscription(Image,'/image2',self.image2_callback,10)
        self.create_subscription(Bool, "/run_feature_correspondence", self.run_callback, 10)
        self.create_subscription(Bool, "/run_feature_correspondence_bonus", self.run_bonus_callback, 10)
        self.state_message=self.create_publisher(String, '/robot_report',qos_profile=10 )
        self.completion_publisher = self.create_publisher(Bool, '/feature_correspondence_complete', 10)  # Add this
        self.run=False
        self.run_bonus=False
        self.img1=None
        self.img2=None
        
        # Create timer to periodically check and process images
        self.timer = self.create_timer(0.1, self.process_images)
        self.get_logger().info('Feature Correspondence Node initialized successfully')
        self.get_logger().info('Subscribed to: /image1, /image2, /run_feature_correspondence, /run_feature_correspondence_bonus')
        self.get_logger().info('Publishing to: /robot_report')

    def image1_callback(self,msg):
        self.get_logger().debug('Received image1')
        try:
            self.img1=self.bridge.imgmsg_to_cv2(msg,"bgr8")
            self.get_logger().info(f'Image1 received: shape={self.img1.shape}, dtype={self.img1.dtype}')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image1: {e}')

    def image2_callback(self,msg):
        self.get_logger().debug('Received image2')
        try:
            self.img2=self.bridge.imgmsg_to_cv2(msg,"bgr8")
            self.get_logger().info(f'Image2 received: shape={self.img2.shape}, dtype={self.img2.dtype}')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image2: {e}')
        
    def run_callback(self,msg):
        self.get_logger().info(f'Received run_feature_correspondence message: {msg.data}')
        if msg.data:
            self.run=True
            self.get_logger().info("Feature correspondence ENABLED - will process on next cycle")
        else:
            self.run=False
            self.get_logger().info("Feature correspondence DISABLED")

    def run_bonus_callback(self,msg):
        self.get_logger().info(f'Received run_feature_correspondence_bonus message: {msg.data}')
        if msg.data:
            self.run_bonus=True
            self.get_logger().info("Feature correspondence BONUS (LightGlue) ENABLED")
        else:
            self.run_bonus=False
            self.get_logger().info("Feature correspondence BONUS DISABLED")
        
    def process_images(self):
        if self.run:
            self.get_logger().info('=== Starting basic feature correspondence processing ===')
            
            if self.img1 is None or self.img2 is None:
                self.get_logger().warn(f"Cannot process: img1={self.img1 is not None}, img2={self.img2 is not None}")
                return
            
            self.get_logger().info(f'Processing images: img1 shape={self.img1.shape}, img2 shape={self.img2.shape}')
            
            try:
                self.get_logger().debug('Converting to grayscale...')
                gray1 = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)
                self.get_logger().debug(f'Grayscale conversion complete: gray1={gray1.shape}, gray2={gray2.shape}')
                
                self.get_logger().debug('Applying threshold...')
                thresh1=cv2.threshold(gray1,50,255,cv2.THRESH_BINARY)[1]
                thresh2=cv2.threshold(gray2,50,255,cv2.THRESH_BINARY)[1]
                self.get_logger().debug('Threshold applied')
                
                self.get_logger().debug('Calculating moments for image 1...')
                center1=cv2.moments(thresh1, binaryImage=True)

                if center1["m00"] != 0:
                    c1X = int(center1["m10"] / center1["m00"])
                    c1Y = int(center1["m01"] / center1["m00"])
                    self.get_logger().info(f'Image1 centroid: ({c1X}, {c1Y}), m00={center1["m00"]}')
                else:
                    c1X, c1Y = 0, 0
                    self.get_logger().warn('Image1 centroid: m00 is zero, defaulting to (0,0)')
                
                self.get_logger().debug('Calculating moments for image 2...')
                center2=cv2.moments(thresh2, binaryImage=True)

                if center2["m00"] != 0:
                    c2X = int(center2["m10"] / center2["m00"])
                    c2Y = int(center2["m01"] / center2["m00"])
                    self.get_logger().info(f'Image2 centroid: ({c2X}, {c2Y}), m00={center2["m00"]}')
                else:
                    c2X, c2Y = 0, 0
                    self.get_logger().warn('Image2 centroid: m00 is zero, defaulting to (0,0)')

                dx = c2X - c1X
                dy = c2Y - c1Y
                self.get_logger().info(f'Calculated movement: dx={dx}, dy={dy}')
                
                msg_str=String()
                msg_str.data = f"Movement at x: {dx}, y: {dy}"
                self.state_message.publish(msg_str)
                self.get_logger().info(f'Published result: "{msg_str.data}"')
                
                self.img1=None
                self.img2=None
                self.run=False
                self.get_logger().info('=== Basic feature correspondence complete, images cleared, run flag reset ===')
                
                # Signal completion
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().info('Published completion signal to /feature_correspondence_complete')
                
            except Exception as e:
                self.get_logger().error(f'Error during basic processing: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                # Signal completion even on error so state machine doesn't hang
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                
        elif self.run_bonus:
            self.get_logger().info('=== Starting LightGlue feature correspondence processing ===')
            
            if self.img1 is None or self.img2 is None:
                self.get_logger().warn(f"Cannot process bonus: img1={self.img1 is not None}, img2={self.img2 is not None}")
                return
            
            try:
                self.get_logger().info(f'Processing with LightGlue: img1 shape={self.img1.shape}, img2 shape={self.img2.shape}')
                
                img0 = self.img1
                img1 = self.img2

                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.get_logger().info(f'Using device: {device}')
                
                self.get_logger().debug('Converting images to tensors...')
                img0 = torch.from_numpy(img0).float() / 255.0
                img1 = torch.from_numpy(img1).float() / 255.0
                self.get_logger().debug(f'Tensor conversion complete: img0={img0.shape}, img1={img1.shape}')
                
                def prepare_image(t):
                    if t.ndim == 2:
                        t = t.unsqueeze(0).unsqueeze(0)
                    if t.ndim == 3:
                        t = t.unsqueeze(0)
                    if t.shape[1] == 1:
                        t = t.repeat(1, 3, 1, 1)
                    return t.to(device)

                img0 = prepare_image(img0)
                img1 = prepare_image(img1)
                self.get_logger().info(f'Images prepared: img0={img0.shape}, img1={img1.shape}')

                def resize_img(t, max_size=1024):
                    _, _, h, w = t.shape
                    if max(h, w) > max_size:
                        scale = max_size / max(h, w)
                        new_h, new_w = int(h * scale), int(w * scale)
                        self.get_logger().debug(f'Resizing from ({h},{w}) to ({new_h},{new_w})')
                        t = torch.nn.functional.interpolate(t, size=(new_h, new_w),
                                                            mode="bilinear", align_corners=False)
                    return t

                img0 = resize_img(img0)
                img1 = resize_img(img1)
                self.get_logger().info(f'After resize: img0={img0.shape}, img1={img1.shape}')

                self.get_logger().info('Loading SuperPoint and LightGlue models...')
                extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
                matcher = LightGlue(
                    features="superpoint",
                    depth_confidence=0.9,
                    width_confidence=0.9,
                ).eval().to(device)
                self.get_logger().info('Models loaded successfully')

                self.get_logger().info('Running inference...')
                with torch.no_grad():
                    feats0 = extractor.extract(img0)
                    self.get_logger().debug(f'Extracted features from img0: {len(feats0["keypoints"])} keypoints')
                    feats1 = extractor.extract(img1)
                    self.get_logger().debug(f'Extracted features from img1: {len(feats1["keypoints"])} keypoints')
                    matches_data = matcher({"image0": feats0, "image1": feats1})
                    self.get_logger().debug('Matching complete')

                feats0, feats1, matches_data = rbd(feats0), rbd(feats1), rbd(matches_data)

                matches0 = matches_data["matches0"].cpu()
                if matches0.ndim == 2:
                    matches0 = matches0[0]
                matches0 = matches0.numpy()

                kpts0 = feats0["keypoints"].cpu()
                if kpts0.ndim == 3:
                    kpts0 = kpts0[0]
                kpts0 = kpts0.numpy()

                kpts1 = feats1["keypoints"].cpu()
                if kpts1.ndim == 3:
                    kpts1 = kpts1[0]
                kpts1 = kpts1.numpy()

                self.get_logger().info(f'Keypoints: img0={len(kpts0)}, img1={len(kpts1)}')

                good_matches = []
                for i, j in enumerate(matches0):
                    if j >= 0:
                        m = cv2.DMatch()
                        m.queryIdx = i
                        m.trainIdx = int(j)
                        m.distance = 0.0
                        good_matches.append(m)

                self.get_logger().info(f"SuperPoint + LightGlue found {len(good_matches)} matches!")

                left  = self.img1
                right = self.img2

                kp0 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts0]
                kp1 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts1]

                h0, w0 = img0.shape[2], img0.shape[3]
                h1, w1 = img1.shape[2], img1.shape[3]

                left_vis  = cv2.resize(left,  (w0, h0))
                right_vis = cv2.resize(right, (w1, h1))

                result = cv2.drawMatches(
                    left_vis, kp0,
                    right_vis, kp1,
                    good_matches, None,
                    matchColor=(0, 255, 0),
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )

                self.get_logger().info('Displaying result window...')
                cv2.imshow("SuperPoint + LightGlue – 2025 (finally flawless)", result)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                self.get_logger().info('Window closed')
                
                self.img1 = None
                self.img2 = None
                self.run_bonus = False
                self.get_logger().info('=== LightGlue feature correspondence complete ===')
                
                # Signal completion
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().info('Published completion signal to /feature_correspondence_complete')
                
            except Exception as e:
                self.get_logger().error(f'Error during LightGlue processing: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                # Signal completion even on error
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                  
def main(args=None):
    rclpy.init(args=args)
    node = FeatureCorrespondence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


