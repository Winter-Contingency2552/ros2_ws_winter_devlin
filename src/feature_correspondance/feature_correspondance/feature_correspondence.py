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
        super().__init__('feature_correspondance_node')
        self.get_logger().info('========================================')
        self.get_logger().info('Initializing Feature Correspondence Node...')
        self.get_logger().info('========================================')
        self.bridge=CvBridge()
        self.camera_sub1=self.create_subscription(Image,'/image1',self.image1_callback,10)
        self.camera_sub2=self.create_subscription(Image,'/image2',self.image2_callback,10)
        self.create_subscription(Bool, "/run_feature_correspondence", self.run_callback, 10)
        self.create_subscription(Bool, "/run_feature_correspondence_bonus", self.run_bonus_callback, 10)
        self.state_message=self.create_publisher(String, '/robot_report',qos_profile=10 )
        self.completion_publisher = self.create_publisher(Bool, '/feature_correspondence_complete', 10)
        self.run=False
        self.run_bonus=False
        self.img1=None
        self.img2=None
        
        # Counters to track how many images have been received
        self.img1_count = 0
        self.img2_count = 0
        
        # Create timer to periodically check and process images
        self.timer = self.create_timer(0.1, self.process_images)
        
        # Create a status timer to periodically log the node's state
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info('Feature Correspondence Node initialized successfully')
        self.get_logger().info('Subscribed to: /image1, /image2, /run_feature_correspondence, /run_feature_correspondence_bonus')
        self.get_logger().info('Publishing to: /robot_report, /feature_correspondence_complete')

    def log_status(self):
        """Periodically log the node's current state for debugging"""
        img1_status = f"shape={self.img1.shape}" if self.img1 is not None else "None"
        img2_status = f"shape={self.img2.shape}" if self.img2 is not None else "None"
        self.get_logger().info(f'[STATUS] run={self.run}, run_bonus={self.run_bonus}')
        self.get_logger().info(f'[STATUS] img1={img1_status} (received {self.img1_count} times)')
        self.get_logger().info(f'[STATUS] img2={img2_status} (received {self.img2_count} times)')

    def image1_callback(self, msg):
        self.img1_count += 1
        self.get_logger().info('========================================')
        self.get_logger().info(f'>>> IMAGE1 CALLBACK #{self.img1_count}')
        self.get_logger().info(f'    Message header frame_id: {msg.header.frame_id}')
        self.get_logger().info(f'    Message encoding: {msg.encoding}')
        self.get_logger().info(f'    Message dimensions: {msg.width}x{msg.height}')
        self.get_logger().info(f'    Message step: {msg.step}')
        self.get_logger().info(f'    Message data length: {len(msg.data)}')
        self.get_logger().info('========================================')
        try:
            self.img1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f'Image1 SUCCESSFULLY stored: shape={self.img1.shape}, dtype={self.img1.dtype}')
            self.get_logger().info(f'Image1 pixel range: min={self.img1.min()}, max={self.img1.max()}')
        except Exception as e:
            self.get_logger().error(f'FAILED to convert image1: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def image2_callback(self, msg):
        self.img2_count += 1
        self.get_logger().info('========================================')
        self.get_logger().info(f'>>> IMAGE2 CALLBACK #{self.img2_count}')
        self.get_logger().info(f'    Message header frame_id: {msg.header.frame_id}')
        self.get_logger().info(f'    Message encoding: {msg.encoding}')
        self.get_logger().info(f'    Message dimensions: {msg.width}x{msg.height}')
        self.get_logger().info(f'    Message step: {msg.step}')
        self.get_logger().info(f'    Message data length: {len(msg.data)}')
        self.get_logger().info('========================================')
        try:
            self.img2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f'Image2 SUCCESSFULLY stored: shape={self.img2.shape}, dtype={self.img2.dtype}')
            self.get_logger().info(f'Image2 pixel range: min={self.img2.min()}, max={self.img2.max()}')
        except Exception as e:
            self.get_logger().error(f'FAILED to convert image2: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
    def run_callback(self,msg):
        self.get_logger().info('========================================')
        self.get_logger().info(f'>>> RECEIVED run_feature_correspondence: {msg.data}')
        self.get_logger().info('========================================')
        if msg.data:
            self.run=True
            self.get_logger().info("Feature correspondence ENABLED - will process on next timer cycle")
            self.get_logger().info(f"Current image status: img1={self.img1 is not None}, img2={self.img2 is not None}")
        else:
            self.run=False
            self.get_logger().info("Feature correspondence DISABLED")

    def run_bonus_callback(self,msg):
        self.get_logger().info('========================================')
        self.get_logger().info(f'>>> RECEIVED run_feature_correspondence_bonus: {msg.data}')
        self.get_logger().info('========================================')
        if msg.data:
            self.run_bonus=True
            self.get_logger().info("Feature correspondence BONUS (LightGlue) ENABLED")
            self.get_logger().info(f"Current image status: img1={self.img1 is not None}, img2={self.img2 is not None}")
        else:
            self.run_bonus=False
            self.get_logger().info("Feature correspondence BONUS DISABLED")
        
    def process_images(self):
        # Log every call to see if timer is running (but only when flags are set)
        if self.run or self.run_bonus:
            self.get_logger().info(f'[TIMER] process_images called: run={self.run}, run_bonus={self.run_bonus}')
        
        if self.run:
            self.get_logger().info('=== Starting basic feature correspondence processing ===')
            
            if self.img1 is None or self.img2 is None:
                self.get_logger().warn(f"Cannot process: img1={'OK' if self.img1 is not None else 'MISSING'}, img2={'OK' if self.img2 is not None else 'MISSING'}")
                self.get_logger().warn("Waiting for both images to be received...")
                return
            
            self.get_logger().info(f'Both images available! Processing: img1 shape={self.img1.shape}, img2 shape={self.img2.shape}')
            
            # Signal that processing is in progress (not complete)
            complete_msg = Bool()
            complete_msg.data = False
            self.completion_publisher.publish(complete_msg)
            self.get_logger().info('Published False to /feature_correspondence_complete (processing started)')
            
            try:
                # Display raw images received
                self.get_logger().info('Displaying raw images received...')
                cv2.imshow("Raw Image 1 (img1)", self.img1)
                cv2.imshow("Raw Image 2 (img2)", self.img2)
                
                # Also show them side by side
                h1, w1 = self.img1.shape[:2]
                h2, w2 = self.img2.shape[:2]
                max_h = max(h1, h2)
                # Resize images to same height for side-by-side display
                if h1 != max_h:
                    scale = max_h / h1
                    img1_resized = cv2.resize(self.img1, (int(w1 * scale), max_h))
                else:
                    img1_resized = self.img1.copy()
                if h2 != max_h:
                    scale = max_h / h2
                    img2_resized = cv2.resize(self.img2, (int(w2 * scale), max_h))
                else:
                    img2_resized = self.img2.copy()
                
                side_by_side = np.hstack([img1_resized, img2_resized])
                cv2.imshow("Raw Images Side-by-Side (Left: img1, Right: img2)", side_by_side)
                
                self.get_logger().info('Raw images displayed. Press any key to continue processing...')
                cv2.waitKey(0)
                cv2.destroyWindow("Raw Image 1 (img1)")
                cv2.destroyWindow("Raw Image 2 (img2)")
                cv2.destroyWindow("Raw Images Side-by-Side (Left: img1, Right: img2)")
                self.get_logger().info('Raw image windows closed, continuing with processing...')
                
                self.get_logger().info('Converting to grayscale...')
                gray1 = cv2.cvtColor(self.img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)
                self.get_logger().info(f'Grayscale conversion complete: gray1={gray1.shape}, gray2={gray2.shape}')
                
                self.get_logger().info('Applying threshold...')
                thresh1=cv2.threshold(gray1,50,255,cv2.THRESH_BINARY)[1]
                thresh2=cv2.threshold(gray2,50,255,cv2.THRESH_BINARY)[1]
                self.get_logger().info('Threshold applied')
                
                self.get_logger().info('Calculating moments for image 1...')
                center1=cv2.moments(thresh1, binaryImage=True)

                if center1["m00"] != 0:
                    c1X = int(center1["m10"] / center1["m00"])
                    c1Y = int(center1["m01"] / center1["m00"])
                    self.get_logger().info(f'Image1 centroid: ({c1X}, {c1Y}), m00={center1["m00"]}')
                else:
                    c1X, c1Y = 0, 0
                    self.get_logger().warn('Image1 centroid: m00 is zero, defaulting to (0,0)')
                
                self.get_logger().info('Calculating moments for image 2...')
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
                self.get_logger().info(f'Published result to /robot_report: "{msg_str.data}"')
                
                self.img1=None
                self.img2=None
                self.run=False
                self.get_logger().info('=== Basic feature correspondence complete, images cleared, run flag reset ===')
                
                # Signal completion
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().info('========================================')
                self.get_logger().info('Published TRUE to /feature_correspondence_complete')
                self.get_logger().info('========================================')
                
            except Exception as e:
                self.get_logger().error(f'Error during basic processing: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                # Signal completion even on error so state machine doesn't hang
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().warn('Published TRUE to /feature_correspondence_complete (despite error)')
                self.run = False
                
        elif self.run_bonus:
            self.get_logger().info('=== Starting LightGlue feature correspondence processing ===')
            self.get_logger().info(f'Timestamp: {self.get_clock().now().to_msg()}')
            
            if self.img1 is None or self.img2 is None:
                self.get_logger().warn(f"Cannot process bonus: img1={'OK' if self.img1 is not None else 'MISSING'}, img2={'OK' if self.img2 is not None else 'MISSING'}")
                self.get_logger().warn("Waiting for both images to be received...")
                return
            
            # Signal that processing is in progress (not complete)
            complete_msg = Bool()
            complete_msg.data = False
            self.completion_publisher.publish(complete_msg)
            
            try:
                self.get_logger().info(f'Processing with LightGlue: img1 shape={self.img1.shape}, img2 shape={self.img2.shape}')
                self.get_logger().info(f'Image1 dtype: {self.img1.dtype}, min={self.img1.min()}, max={self.img1.max()}')
                self.get_logger().info(f'Image2 dtype: {self.img2.dtype}, min={self.img2.min()}, max={self.img2.max()}')
                
                # Display raw images received
                self.get_logger().info('Displaying raw images received...')
                cv2.imshow("Raw Image 1 (img1)", self.img1)
                cv2.imshow("Raw Image 2 (img2)", self.img2)
                
                # Also show them side by side
                h1, w1 = self.img1.shape[:2]
                h2, w2 = self.img2.shape[:2]
                max_h = max(h1, h2)
                # Resize images to same height for side-by-side display
                if h1 != max_h:
                    scale = max_h / h1
                    img1_resized = cv2.resize(self.img1, (int(w1 * scale), max_h))
                else:
                    img1_resized = self.img1.copy()
                if h2 != max_h:
                    scale = max_h / h2
                    img2_resized = cv2.resize(self.img2, (int(w2 * scale), max_h))
                else:
                    img2_resized = self.img2.copy()
                
                side_by_side = np.hstack([img1_resized, img2_resized])
                cv2.imshow("Raw Images Side-by-Side (Left: img1, Right: img2)", side_by_side)
                
                self.get_logger().info('Raw images displayed. Press any key to continue processing...')
                cv2.waitKey(0)
                cv2.destroyWindow("Raw Image 1 (img1)")
                cv2.destroyWindow("Raw Image 2 (img2)")
                cv2.destroyWindow("Raw Images Side-by-Side (Left: img1, Right: img2)")
                self.get_logger().info('Raw image windows closed, continuing with processing...')
                
                img0 = self.img1
                img1 = self.img2
                if img0 is None or img1 is None:
                    self.get_logger().error('One of the images is None, aborting LightGlue processing')
                    return
                
                self.get_logger().info('Checking CUDA availability...')
                self.get_logger().info(f'CUDA available: {torch.cuda.is_available()}')
                if torch.cuda.is_available():
                    self.get_logger().info(f'CUDA device count: {torch.cuda.device_count()}')
                    self.get_logger().info(f'CUDA device name: {torch.cuda.get_device_name(0)}')
                    self.get_logger().info(f'CUDA memory allocated: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB')
                    self.get_logger().info(f'CUDA memory cached: {torch.cuda.memory_reserved(0) / 1024**2:.2f} MB')
                
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.get_logger().info(f'Using device: {device}')
                
                self.get_logger().info('Converting images to tensors...')
                self.get_logger().debug(f'img0 numpy shape before conversion: {img0.shape}')
                self.get_logger().debug(f'img1 numpy shape before conversion: {img1.shape}')
                
                img0 = torch.from_numpy(img0).float() / 255.0
                img1 = torch.from_numpy(img1).float() / 255.0
                
                self.get_logger().info(f'Tensor conversion complete: img0={img0.shape}, img1={img1.shape}')
                self.get_logger().info(f'img0 tensor dtype: {img0.dtype}, min={img0.min().item():.4f}, max={img0.max().item():.4f}')
                self.get_logger().info(f'img1 tensor dtype: {img1.dtype}, min={img1.min().item():.4f}, max={img1.max().item():.4f}')
                
                def prepare_image(t):
                    self.get_logger().debug(f'prepare_image input shape: {t.shape}, ndim: {t.ndim}')
                    if t.ndim == 2:
                        self.get_logger().debug('Image is 2D, adding batch and channel dimensions')
                        t = t.unsqueeze(0).unsqueeze(0)
                    if t.ndim == 3:
                        self.get_logger().debug('Image is 3D, adding batch dimension')
                        t = t.unsqueeze(0)
                    if t.shape[1] == 1:
                        self.get_logger().debug('Image has 1 channel, repeating to 3 channels')
                        t = t.repeat(1, 3, 1, 1)
                    self.get_logger().debug(f'prepare_image output shape: {t.shape}')
                    return t.to(device)

                self.get_logger().info('Preparing image 0...')
                img0 = prepare_image(img0)
                self.get_logger().info('Preparing image 1...')
                img1 = prepare_image(img1)
                self.get_logger().info(f'Images prepared and moved to device: img0={img0.shape}, img1={img1.shape}')
                self.get_logger().info(f'img0 device: {img0.device}, img1 device: {img1.device}')

                def resize_img(t, max_size=1024):
                    _, _, h, w = t.shape
                    self.get_logger().debug(f'resize_img input: h={h}, w={w}, max_size={max_size}')
                    if max(h, w) > max_size:
                        scale = max_size / max(h, w)
                        new_h, new_w = int(h * scale), int(w * scale)
                        self.get_logger().info(f'Resizing from ({h},{w}) to ({new_h},{new_w}), scale={scale:.4f}')
                        t = torch.nn.functional.interpolate(t, size=(new_h, new_w),
                                                            mode="bilinear", align_corners=False)
                    else:
                        self.get_logger().debug(f'No resize needed, image within max_size')
                    return t

                self.get_logger().info('Resizing image 0 if needed...')
                img0 = resize_img(img0)
                self.get_logger().info('Resizing image 1 if needed...')
                img1 = resize_img(img1)
                self.get_logger().info(f'After resize: img0={img0.shape}, img1={img1.shape}')

                self.get_logger().info('Loading SuperPoint model...')
                import time
                start_time = time.time()
                extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
                superpoint_load_time = time.time() - start_time
                self.get_logger().info(f'SuperPoint model loaded in {superpoint_load_time:.3f} seconds')
                
                self.get_logger().info('Loading LightGlue model...')
                start_time = time.time()
                matcher = LightGlue(
                    features="superpoint",
                    depth_confidence=0.9,
                    width_confidence=0.9,
                ).eval().to(device)
                lightglue_load_time = time.time() - start_time
                self.get_logger().info(f'LightGlue model loaded in {lightglue_load_time:.3f} seconds')
                self.get_logger().info('Both models loaded successfully')

                if torch.cuda.is_available():
                    self.get_logger().info(f'GPU memory after model load: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB')

                self.get_logger().info('Running feature extraction on image 0...')
                start_time = time.time()
                with torch.no_grad():
                    feats0 = extractor.extract(img0)
                feat0_time = time.time() - start_time
                self.get_logger().info(f'Image 0 feature extraction completed in {feat0_time:.3f} seconds')
                self.get_logger().info(f'Image 0 keypoints shape: {feats0["keypoints"].shape}')
                self.get_logger().info(f'Image 0 descriptors shape: {feats0["descriptors"].shape if "descriptors" in feats0 else "N/A"}')
                
                self.get_logger().info('Running feature extraction on image 1...')
                start_time = time.time()
                with torch.no_grad():
                    feats1 = extractor.extract(img1)
                feat1_time = time.time() - start_time
                self.get_logger().info(f'Image 1 feature extraction completed in {feat1_time:.3f} seconds')
                self.get_logger().info(f'Image 1 keypoints shape: {feats1["keypoints"].shape}')
                self.get_logger().info(f'Image 1 descriptors shape: {feats1["descriptors"].shape if "descriptors" in feats1 else "N/A"}')
                
                self.get_logger().info('Running LightGlue matching...')
                start_time = time.time()
                with torch.no_grad():
                    matches_data = matcher({"image0": feats0, "image1": feats1})
                match_time = time.time() - start_time
                self.get_logger().info(f'Matching completed in {match_time:.3f} seconds')
                self.get_logger().info(f'matches_data keys: {matches_data.keys()}')

                self.get_logger().info('Removing batch dimension from features and matches...')
                feats0, feats1, matches_data = rbd(feats0), rbd(feats1), rbd(matches_data)
                self.get_logger().info('Batch dimension removed')

                self.get_logger().info('Processing matches0...')
                matches0 = matches_data["matches0"].cpu()
                self.get_logger().debug(f'matches0 shape before squeeze: {matches0.shape}, ndim: {matches0.ndim}')
                if matches0.ndim == 2:
                    matches0 = matches0[0]
                    self.get_logger().debug(f'matches0 squeezed to shape: {matches0.shape}')
                matches0 = matches0.numpy()
                self.get_logger().info(f'matches0 numpy array shape: {matches0.shape}')
                self.get_logger().info(f'matches0 valid matches (>=0): {(matches0 >= 0).sum()}')
                self.get_logger().info(f'matches0 invalid matches (<0): {(matches0 < 0).sum()}')

                self.get_logger().info('Processing keypoints from image 0...')
                kpts0 = feats0["keypoints"].cpu()
                self.get_logger().debug(f'kpts0 shape before squeeze: {kpts0.shape}, ndim: {kpts0.ndim}')
                if kpts0.ndim == 3:
                    kpts0 = kpts0[0]
                    self.get_logger().debug(f'kpts0 squeezed to shape: {kpts0.shape}')
                kpts0 = kpts0.numpy()
                self.get_logger().info(f'kpts0 numpy array shape: {kpts0.shape}')

                self.get_logger().info('Processing keypoints from image 1...')
                kpts1 = feats1["keypoints"].cpu()
                self.get_logger().debug(f'kpts1 shape before squeeze: {kpts1.shape}, ndim: {kpts1.ndim}')
                if kpts1.ndim == 3:
                    kpts1 = kpts1[0]
                    self.get_logger().debug(f'kpts1 squeezed to shape: {kpts1.shape}')
                kpts1 = kpts1.numpy()
                self.get_logger().info(f'kpts1 numpy array shape: {kpts1.shape}')

                self.get_logger().info(f'Total keypoints: img0={len(kpts0)}, img1={len(kpts1)}')
                
                if len(kpts0) > 0:
                    self.get_logger().info(f'kpts0 coordinate range: x=[{kpts0[:,0].min():.2f}, {kpts0[:,0].max():.2f}], y=[{kpts0[:,1].min():.2f}, {kpts0[:,1].max():.2f}]')
                if len(kpts1) > 0:
                    self.get_logger().info(f'kpts1 coordinate range: x=[{kpts1[:,0].min():.2f}, {kpts1[:,0].max():.2f}], y=[{kpts1[:,1].min():.2f}, {kpts1[:,1].max():.2f}]')

                self.get_logger().info('Building good_matches list...')
                good_matches = []
                for i, j in enumerate(matches0):
                    if j >= 0:
                        m = cv2.DMatch()
                        m.queryIdx = i
                        m.trainIdx = int(j)
                        m.distance = 0.0
                        good_matches.append(m)

                self.get_logger().info(f"SuperPoint + LightGlue found {len(good_matches)} matches!")
                self.get_logger().info(f"Match rate: {len(good_matches)/len(kpts0)*100:.2f}% of img0 keypoints matched")
                
                if len(good_matches) > 0:
                    self.get_logger().info(f'First 5 matches (queryIdx -> trainIdx):')
                    for idx, m in enumerate(good_matches[:5]):
                        self.get_logger().info(f'  Match {idx}: kpt0[{m.queryIdx}] -> kpt1[{m.trainIdx}]')

                self.get_logger().info('Preparing visualization...')
                left  = self.img1
                right = self.img2

                self.get_logger().debug('Converting keypoints to cv2.KeyPoint objects...')
                kp0 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts0]
                kp1 = [cv2.KeyPoint(x=pt[0], y=pt[1], size=6) for pt in kpts1]
                self.get_logger().info(f'Created {len(kp0)} KeyPoints for img0, {len(kp1)} KeyPoints for img1')

                h0, w0 = img0.shape[2], img0.shape[3]
                h1, w1 = img1.shape[2], img1.shape[3]
                self.get_logger().info(f'Visualization dimensions: img0=({w0}x{h0}), img1=({w1}x{h1})')

                self.get_logger().debug('Resizing images for visualization...')
                left_vis  = cv2.resize(left,  (w0, h0))
                right_vis = cv2.resize(right, (w1, h1))
                self.get_logger().debug(f'left_vis shape: {left_vis.shape}, right_vis shape: {right_vis.shape}')

                self.get_logger().info('Drawing matches...')
                result = cv2.drawMatches(
                    left_vis, kp0,
                    right_vis, kp1,
                    good_matches, None,
                    matchColor=(0, 255, 0),
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )
                self.get_logger().info(f'Result image shape: {result.shape}')

                self.get_logger().info('Displaying result window...')
                cv2.imshow("SuperPoint + LightGlue – 2025 (finally flawless)", result)
                self.get_logger().info('Waiting for key press to close window...')
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                self.get_logger().info('Window closed')
                
                # Log timing summary
                total_time = superpoint_load_time + lightglue_load_time + feat0_time + feat1_time + match_time
                self.get_logger().info('=== Timing Summary ===')
                self.get_logger().info(f'  SuperPoint load: {superpoint_load_time:.3f}s')
                self.get_logger().info(f'  LightGlue load: {lightglue_load_time:.3f}s')
                self.get_logger().info(f'  Feature extraction img0: {feat0_time:.3f}s')
                self.get_logger().info(f'  Feature extraction img1: {feat1_time:.3f}s')
                self.get_logger().info(f'  Matching: {match_time:.3f}s')
                self.get_logger().info(f'  Total processing time: {total_time:.3f}s')
                
                if torch.cuda.is_available():
                    self.get_logger().info(f'Final GPU memory usage: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB')
                
                self.img1 = None
                self.img2 = None
                self.run_bonus = False
                self.get_logger().info('Images cleared, run_bonus flag reset')
                self.get_logger().info('=== LightGlue feature correspondence complete ===')
                
                # Signal completion
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().info('Published completion signal to /feature_correspondence_complete')
                
            except Exception as e:
                self.get_logger().error(f'Error during LightGlue processing: {e}')
                self.get_logger().error(f'Error type: {type(e).__name__}')
                import traceback
                self.get_logger().error('Full traceback:')
                self.get_logger().error(traceback.format_exc())
                # Signal completion even on error
                complete_msg = Bool()
                complete_msg.data = True
                self.completion_publisher.publish(complete_msg)
                self.get_logger().warn('Published completion signal despite error')

# --- Added: entrypoint so ros2/console script can spin the node --- #
def main(args=None):
    rclpy.init(args=args)
    node = FeatureCorrespondence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()