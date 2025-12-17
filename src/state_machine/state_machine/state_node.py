import cmd
from itertools import accumulate
from re import search
from textwrap import wrap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math
import re
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from aruco_interfaces.msg import ArucoMarkerArray
from std_msgs.msg import Bool,String

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create publishers
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.search_complete_pub = self.create_publisher(Bool, 'search_complete', 10)
        self.search_ended_pub = self.create_publisher(Bool, 'search_ended', 10)
        self.report_publisher = self.create_publisher(String, '/robot_report', 10)
        self.movement_publisher = self.create_publisher(Bool, '/run_feature_correspondence', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        self.bonus_publisher = self.create_publisher(Bool, '/run_feature_correspondence_bonus', 10) 
        # Create subscribers
        self.sub = self.create_subscription(ArucoMarkerArray, "/aruco/markers", self.marker_callback, 10)
        self.missions_sub = self.create_subscription(String, "/missions", self.missions_callback, 10)
        self.feedback_sub = self.create_subscription(String, "/evaluator_message", self.evaluator_callback, 10)
        self.feature_complete_sub = self.create_subscription(Bool, "/feature_correspondence_complete", self.feature_complete_callback, 10)  # Add this

        # TF2 buffer and listener reading world to base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State Machine Variables
        self.state = 'Initial'
        self.side_index = 0 # 0 & 2: short side, 1 & 3: long side
        self.side_length_wide = 6.5 # meters (long side of desired rectangle)
        self.side_length_narrow = 2.5 # meters (short side of desired rectangle)

        # Establish Yaw Needed To Look At Each Wall
        self.wall1_yaw = 0.0 # radians
        self.wall2_yaw = -math.pi / 2 # radians
        self.wall3_yaw = math.pi # radians
        self.wall4_yaw = math.pi / 2 # radians
        self.wall_yaw = [self.wall1_yaw, self.wall2_yaw, self.wall3_yaw, self.wall4_yaw]
        self.wall_checked_1 = False
        self.wall_checked_2 = False
        self.wall_checked_3 = False
        self.wall_checked_4 = False
        self.Num_markers_detected = 0

        # Establish Goal Points Based On (0,0) Assumed Spawn
        self.gp1 = (self.side_length_wide/2, self.side_length_narrow/2)
        self.gp2 = (self.side_length_wide/2, -self.side_length_narrow/2)
        self.gp3 = (-self.side_length_wide/2, -self.side_length_narrow/2)
        self.gp4 = (-self.side_length_wide/2, self.side_length_narrow/2)
        self.goal_points = [self.gp1, self.gp2, self.gp3, self.gp4]
        self.extra_distance = 0.35 # meters to overshoot each goal point
        self.check_distance = 1.5 # meters to turn to check for markers on wall
        self.speed = 5.0 # m/s
        self.angular_speed = 5.0 # rad/s
        self.accumulated_angle = 0.0
        self.returned_to_start = False

        self.pos_tol = 0.05 # [m] tolerance for position
        self.ang_tol = math.radians(0.1) # [rad] tolerance for angle

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0

        self.checkpoint_x = 0.0
        self.checkpoint_y = 0.0
        self.checkpoint_yaw = 0.0

        self.get_logger().info('Controller Node has been started.')
        self.markerArray = None
        self.target_marker_id = None
        self.marker_location_from_evaluator = None
        self.search=True
        self.image_analysis_sent = False
        self.image_analysis_count = 0
        self.feature_correspondence_complete = False

        # Angle correction variables
        self.evaluator_angle_error = None  # Parsed angle error from evaluator (degrees)
        self.evaluator_message_received = False
        self.marker_goal_x = 0.0  # Store marker goal position
        self.marker_goal_y = 0.0
        self.angle_correction_target_yaw = None  # Target yaw for 5-degree clockwise spin
        self.wall_orientation_complete = False  # Flag to track if initial wall orientation is done
        self.waiting_for_angle_feedback = False  # Flag to track if we're waiting for evaluator

    # ---- Helpful Functions ---- #
    
    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


    @staticmethod
    def wrap_to_pi(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        d = ControllerNode.wrap_to_pi(d)
        return d

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def p_orient(self, current_x, current_y, current_yaw, target_x, target_y):
        # Simple Proportional Controller for Orientation
        kp_angular = 1.0
        target_yaw = math.atan2(target_y - current_y, target_x - current_x)
        angular_speed = kp_angular * self.angle_diff(target_yaw, current_yaw)
        if abs(angular_speed) > self.angular_speed:
            angular_speed = self.angular_speed if angular_speed > 0 else -self.angular_speed
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angular_speed
        return cmd
    
    def pid_navigate(self, current_x, current_y, current_yaw, target_x, target_y):
        # Simple Proportional Controller for Navigation
        kp_linear = 0.5
        kp_angular = 1.0

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        linear_speed = kp_linear * distance
        angular_speed = kp_angular * self.angle_diff(target_yaw, current_yaw)
        if linear_speed > self.speed:
            linear_speed = self.speed
        if abs(angular_speed) > self.angular_speed:
            angular_speed = self.angular_speed if angular_speed > 0 else -self.angular_speed

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        return cmd

    # ---- Primary Loop ---- #
    def marker_callback(self, msg: ArucoMarkerArray):
        # Fist Get Number of Markers Detected
        self.markerArray = msg
        self.Num_markers_detected = len(msg.markers)
        self.get_logger().info(
            f"Got {self.Num_markers_detected} markers in frame '{msg.header.frame_id}' at stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        return self.markerArray

    def missions_callback(self, msg: String):
        if msg.data == "standby":
            self.get_logger().info("Mission received: standby")
            self.state = 'Initial'
        elif msg.data == "search_aruco":
            self.get_logger().info("Mission received: search_aruco")
            self.search=True
        elif msg.data.startswith("move"):
            self.search=False
            # Extract the marker ID from the end of the message (e.g., "move to 30" -> 30)
            parts = msg.data.split()  # Split by whitespace
            if parts and parts[-1].isdigit():
                self.target_marker_id = int(parts[-1])
                self.get_logger().info(f"Mission received: go_to_marker {self.target_marker_id}")
                self.state = 'GoToMarker'
            else:
                self.get_logger().warn(f"Could not parse marker ID from: {msg.data}")
        elif msg.data == "image_analysis":
            self.get_logger().info("Mission received: image_analysis")
            self.state = 'image_analysis'
        elif msg.data == "return to origin":
            self.get_logger().info("Mission received: return_to_start")
            self.state = 'ReturnToStart'
        elif msg.data == "image_analysis2":
            self.get_logger().info("Mission received: image_analysis_bonus")
            self.state = 'feature_counting'
        else:
            self.get_logger().info(f"Unknown mission received: {msg.data}")
            self.search=False

    def evaluator_callback(self, msg: String):
        self.marker_location_from_evaluator = msg.data
        
        # Parse angle error from evaluator message
        # Expected format: "aruco X xy_error=0.XXXm angle=XX.Xdeg PASSED/FAILED"
        match = re.search(r'angle=([\d.]+)deg', msg.data)
        if match:
            try:
                self.evaluator_angle_error = float(match.group(1))
                self.evaluator_message_received = True
                self.get_logger().info(f'Parsed angle_error from evaluator: {self.evaluator_angle_error:.1f} deg')
            except ValueError:
                self.get_logger().warn(f'Could not parse angle value from: {msg.data}')

    def feature_complete_callback(self, msg: Bool):
        """Callback for when feature correspondence completes"""
        if msg.data:
            self.feature_correspondence_complete = True
            self.get_logger().info('Received feature correspondence completion signal')

    def go_to_marker(self, marker_id):

        if not hasattr(self, 'markerArray') or self.markerArray is None:
            self.get_logger().warn('No markers received yet.')
            return None
        
        for marker in self.markerArray.markers:
            if marker.id == marker_id:
                goal_x = marker.pose.pose.position.x
                goal_y = marker.pose.pose.position.y
                self.get_logger().info(f'Found marker {marker_id} at ({goal_x:.2f}, {goal_y:.2f})')
                return (goal_x, goal_y)
        
        self.get_logger().warn(f'Marker {marker_id} not found in detected markers.')
        return None
        
    def control_loop(self):
        try:
            tf = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF lookup failed. Retrying...')
            return
        
        # Extract TF Values (moved outside the if block)
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = self.yaw_from_quaternion(q)
        cmd = Twist()
        
        if self.Num_markers_detected < 6 and self.search:
            search_complete_msg = Bool()
            search_complete_msg.data = False
            search_ended_msg = Bool()
            search_ended_msg.data = False
            self.search_ended_pub.publish(search_ended_msg)
            self.search_complete_pub.publish(search_complete_msg)
            # (removed duplicate TF extraction and cmd initialization)

            # State Machine Logic
            if self.state == 'Initial':
                status_msg = String()
                status_msg.data = 'ready'
                self.status_publisher.publish(status_msg)
                self.init_x = x
                self.init_y = y
                self.init_yaw = yaw
                self.state = 'InitNavigateToGoal'
                self.get_logger().info('State changed to InitNavigateToGoal')

            elif self.state == 'InitNavigateToGoal':
                goal_x, goal_y = self.goal_points[self.side_index]
                pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                dx = goal_x - x
                dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'OrientCorner'
                    self.get_logger().info('Reached Goal Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to OrientCorner')
                else:
                    cmd.linear.x = pid_navigate_cmd.linear.x
                    cmd.angular.z = pid_navigate_cmd.angular.z
            
            elif self.state == 'NavigateToGoal':
                goal_x, goal_y = self.goal_points[self.side_index]
                midpoint_x = (self.checkpoint_x + goal_x) / 2
                midpoint_y = (self.checkpoint_y + goal_y) / 2
                
                if self.wall_checked_1 is False:
                    pid_navigate_cmd = self.pid_navigate(x, y, yaw, midpoint_x, midpoint_y)
                    dx = midpoint_x - x
                    dy = midpoint_y - y
                else:
                    pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                    dx = goal_x - x
                    dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    if self.wall_checked_1 is False:
                        self.wall_checked_1 = True
                        self.get_logger().info('Reached Midpoint At ({:.2f}, {:.2f}), preparing to check wall.'.format(midpoint_x, midpoint_y))
                        self.state = 'WallCheck'
                        self.get_logger().info('State changed to WallCheck')
                    elif self.wall_checked_2 is False:
                        self.wall_checked_2 = True
                        self.get_logger().info('Reached Goal Point At ({:.2f}, {:.2f}), preparing to check wall.'.format(goal_x, goal_y))
                        self.state = 'WallCheck'
                        self.get_logger().info('State changed to WallCheck')
                    else:
                        self.state = 'OrientCorner'
                        self.wall_checked_1 = False
                        self.wall_checked_2 = False
                        self.checkpoint_x = x
                        self.checkpoint_y = y
                        self.checkpoint_yaw = yaw
                        self.get_logger().info('Reached Goal Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                        self.get_logger().info('State changed to OrientCorner')
                else:
                    cmd.linear.x = pid_navigate_cmd.linear.x
                    cmd.angular.z = pid_navigate_cmd.angular.z

            elif self.state == 'WallCheck':
                target_yaw = self.wall_yaw[self.side_index-1]
                p_orient_cmd = self.p_orient(x, y, yaw, x + math.cos(target_yaw), y + math.sin(target_yaw))
                yaw_diff = self.angle_diff(target_yaw, yaw)
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.get_logger().info('Wall Check Complete. Resuming navigation.')
                    self.state = 'ReOrientToGoal'
                    self.get_logger().info('State changed to ReOrientToGoal')
                else:
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z

            elif self.state == 'OrientCorner':
                goal_x, goal_y = self.goal_points[self.side_index]
                if self.side_index == 0:
                    goal_x += self.extra_distance
                    goal_y += self.extra_distance
                elif self.side_index == 1:
                    goal_x += self.extra_distance
                    goal_y -= self.extra_distance
                elif self.side_index == 2:
                    goal_x -= self.extra_distance
                    goal_y -= self.extra_distance
                elif self.side_index == 3:
                    goal_x -= self.extra_distance
                    goal_y += self.extra_distance
                target_yaw = math.atan2(goal_y - y, goal_x - x)
                p_orient_cmd = self.p_orient(x, y, yaw, goal_x, goal_y)
                yaw_diff = self.angle_diff(target_yaw, yaw)
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'CornerCheck'
                    self.get_logger().info('Oriented to Corner Check Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to CornerCheck')
                else:
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z

            elif self.state == 'CornerCheck':
                goal_x, goal_y = self.goal_points[self.side_index]
                if self.side_index == 0:
                    goal_x += self.extra_distance
                    goal_y += self.extra_distance
                elif self.side_index == 1:
                    goal_x += self.extra_distance
                    goal_y -= self.extra_distance
                elif self.side_index == 2:
                    goal_x -= self.extra_distance
                    goal_y -= self.extra_distance
                elif self.side_index == 3:
                    goal_x -= self.extra_distance
                    goal_y += self.extra_distance
                pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                dx = goal_x - x
                dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'OrientToGoal'
                    self.get_logger().info('Reached Corner Check Point at ({:.2f}, {:.2f}). Going Back.'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to OrientToGoal')
                else:
                    cmd.linear.x = pid_navigate_cmd.linear.x
                    cmd.angular.z = pid_navigate_cmd.angular.z

            elif self.state == 'ReOrientToGoal':
                goal_x, goal_y = self.goal_points[self.side_index]
                p_orient_cmd = self.p_orient(x, y, yaw, goal_x, goal_y)
                target_yaw = math.atan2(goal_y - y, goal_x - x)
                yaw_diff = self.angle_diff(target_yaw, yaw)
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.state = 'NavigateToGoal'
                    self.get_logger().info('Oriented to Goal Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to NavigateToGoal')
                else:
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z
            
            elif self.state == 'OrientToGoal':
                goal_x, goal_y = self.goal_points[self.side_index]
                p_orient_cmd = self.p_orient(x, y, yaw, goal_x, goal_y)
                target_yaw = math.atan2(goal_y - y, goal_x - x)
                yaw_diff = self.angle_diff(target_yaw, yaw)
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'ReturnToGoal'
                    self.get_logger().info('Oriented to Goal Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to ReturnToGoal')
                else:
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z

            elif self.state == 'ReturnToGoal':
                goal_x, goal_y = self.goal_points[self.side_index]
                pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                dx = goal_x - x
                dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'Turn'
                    self.side_index += 1
                    self.get_logger().info('Returned to Goal Point At ({:.2f}, {:.2f})'.format(goal_x, goal_y))
                    self.get_logger().info('State changed to Turn')
                else:
                    cmd.linear.x = pid_navigate_cmd.linear.x
                    cmd.angular.z = pid_navigate_cmd.angular.z

            elif self.state == 'Turn':
                self.stop_robot()
                if self.side_index >= 4:
                    self.side_index = 0
                goal_x, goal_y = self.goal_points[self.side_index]
                target_yaw = math.atan2(goal_y - y, goal_x - x)
                p_orient_cmd = self.p_orient(x, y, yaw, goal_x, goal_y)
                yaw_diff = self.angle_diff(target_yaw, yaw)
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.checkpoint_x = x
                    self.checkpoint_y = y
                    self.checkpoint_yaw = yaw
                    self.state = 'NavigateToGoal'
                    self.get_logger().info('State changed to NavigateToGoal')
                else:
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z

            # Publish command
            self.publisher_.publish(cmd)
            
            

        if self.state == 'GoToMarker':
            status_msg = String()
            status_msg.data = 'moving'
            self.status_publisher.publish(status_msg)
            self.get_logger().info('In GoToMarker state')
            if self.target_marker_id is None:
                self.get_logger().warn('No target marker ID specified.')
            else:   
                marker_pos = self.go_to_marker(self.target_marker_id)
                if marker_pos is None:
                    self.get_logger().warn('Target marker not found, skipping.')
                    self.state = 'ReturnToStart'
                else:
                    goal_x, goal_y = marker_pos
                    # Store marker goal position for wall orientation
                    self.marker_goal_x = goal_x
                    self.marker_goal_y = goal_y

                    pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                    dx = goal_x - x
                    dy = goal_y - y
                    distance_to_goal = math.sqrt(dx**2 + dy**2)
                    if distance_to_goal <= self.pos_tol:
                        self.stop_robot()
                        self.get_logger().info(f'Reached marker {self.target_marker_id} at ({goal_x:.2f}, {goal_y:.2f})')
                        report_msg = String()
                        report_msg.data = f'arrived to {self.target_marker_id}'
                        self.report_publisher.publish(report_msg)
                        
                        # Reset angle correction state
                        self.evaluator_message_received = False
                        self.evaluator_angle_error = None
                        self.angle_correction_target_yaw = None
                        self.wall_orientation_complete = False
                        self.waiting_for_angle_feedback = False

                        self.state = 'OrientToWall'
                    else:
                        cmd.linear.x = pid_navigate_cmd.linear.x
                        cmd.angular.z = pid_navigate_cmd.angular.z
                        self.publisher_.publish(cmd)

                        
        elif self.state == 'OrientToWall': # this is the most hacky crap
            self.get_logger().info('=== In OrientToWall state ===')
            self.stop_robot()
            
            # Determine target heading based on marker position
            # x < -2.75: face left wall (heading = -90 deg = -pi/2)
            # y < -4.75: face bottom wall (heading = -180 deg = pi or -pi)
            # x > 2.75: face right wall (heading = 0 deg)
            # y > 4.75: face top wall (heading = 90 deg = pi/2)
            
            if self.marker_goal_x < -2.75:
                target_heading = -math.pi / 2  # -90 degrees
                self.get_logger().info('Orienting to LEFT wall (heading = -90 deg)')
            elif self.marker_goal_y < -4.75:
                target_heading = math.pi  # -180 degrees (same as 180)
                self.get_logger().info('Orienting to BOTTOM wall (heading = -180 deg)')
            elif self.marker_goal_x > 2.75:
                
                target_heading = math.pi / 2  # 90 degrees
                self.get_logger().info('Orienting to RIGHT wall (heading = 90 deg)')
            elif self.marker_goal_y > 4.75:
                target_heading = 0.0  # 0 degrees
                self.get_logger().info('Orienting to TOP wall (heading = 0 deg)')
            else:
                # Default: no specific wall, skip orientation
                self.get_logger().info('Marker not near wall boundary, skipping orientation')
                self.state = 'image_analysis'
                return
            
            # Phase 1: First orient to the predicted wall heading
            if not self.wall_orientation_complete:
                target_x = x + math.cos(target_heading)
                target_y = y + math.sin(target_heading)
                yaw_diff = self.angle_diff(target_heading, yaw)
                
                if abs(yaw_diff) <= self.ang_tol:
                    self.stop_robot()
                    self.get_logger().info(f'Initial wall orientation complete. Current yaw: {math.degrees(yaw):.1f} deg')
                    self.wall_orientation_complete = True
                    # Report position to get angle feedback from evaluator
                    report_msg = String()
                    report_msg.data = f'arrived to {self.target_marker_id}'
                    self.report_publisher.publish(report_msg)
                    self.waiting_for_angle_feedback = True
                    self.evaluator_message_received = False
                else:
                    p_orient_cmd = self.p_orient(x, y, yaw, target_x, target_y)
                    cmd.linear.x = p_orient_cmd.linear.x
                    cmd.angular.z = p_orient_cmd.angular.z
                    self.publisher_.publish(cmd)
                return
            
            # Phase 2: Wait for evaluator feedback
            if self.waiting_for_angle_feedback and not self.evaluator_message_received:
                self.stop_robot()
                self.get_logger().info('Waiting for evaluator angle feedback...')
                return
            
            # Phase 3: Check angle error and spin if needed
            if self.evaluator_message_received:
                if self.evaluator_angle_error is not None and self.evaluator_angle_error > 10.0:
                    report_msg = String()
                    report_msg.data = f'arrived to {self.target_marker_id}'
                    self.report_publisher.publish(report_msg)
                    self.get_logger().info(f'Angle error {self.evaluator_angle_error:.1f} deg > 10 deg, spinning 5 deg clockwise')
                    
                    # If we don't have a target yaw yet, calculate it (5 degrees clockwise = -5 degrees)
                    if self.angle_correction_target_yaw is None:
                        self.angle_correction_target_yaw = self.wrap_to_pi(yaw - math.radians(5.0))
                        self.get_logger().info(f'Set correction target yaw: {math.degrees(self.angle_correction_target_yaw):.1f} deg')
                    
                    # Check if we've reached the target yaw
                    yaw_diff = abs(self.angle_diff(self.angle_correction_target_yaw, yaw))
                    if yaw_diff <= self.ang_tol:
                        # Reached 5-degree increment, report position again to get new angle error
                        self.stop_robot()
                        self.get_logger().info('Completed 5 deg clockwise spin, reporting position')
                        report_msg = String()
                        report_msg.data = f'arrived to {self.target_marker_id}'
                        self.report_publisher.publish(report_msg)
                        # Reset for next evaluation
                        self.evaluator_message_received = False
                        self.angle_correction_target_yaw = None
                    else:
                        # Spin clockwise (negative angular velocity)
                        cmd.linear.x = 0.0
                        cmd.angular.z = -0.5  # Clockwise
                        self.publisher_.publish(cmd)
                    return
                else:
                    # Angle error <= 10 degrees (or None), we're done with orientation
                    error_val = self.evaluator_angle_error if self.evaluator_angle_error is not None else 0.0
                    self.get_logger().info(f'Angle error {error_val:.1f} deg <= 10 deg, orientation complete')
                    self.stop_robot()
                    self.state = 'image_analysis'
                    return

        elif self.state == 'image_analysis':
            self.get_logger().info('=== In image_analysis state ===')
            self.stop_robot()
            
            # Publish status to /robot_status
            status_msg = String()
            status_msg.data = 'analyze image'
            self.status_publisher.publish(status_msg)
            

            
            # Trigger feature correspondence (only once)
            if not self.image_analysis_sent:
                self.get_logger().info('Published "analyze image" to /robot_status')
                run_feature_msg = Bool()
                run_feature_msg.data = True
                self.movement_publisher.publish(run_feature_msg)
                self.get_logger().info('Published True to /run_feature_correspondence')
                self.image_analysis_sent = True
                self.feature_correspondence_complete = False  # Reset flag
            
            # Wait for feature correspondence to complete
            if self.feature_correspondence_complete:
                self.get_logger().info('Feature correspondence complete, transitioning to ReturnToStart')
                self.image_analysis_sent = False  # Reset for next time
                self.image_analysis_count = 0
                self.feature_correspondence_complete = False
                self.state = 'ReturnToStart'
            else:
                self.image_analysis_count += 1
                if self.image_analysis_count % 20 == 0:  # Log every ~1 second
                    self.get_logger().info(f'Waiting for feature correspondence to complete... (count: {self.image_analysis_count})')
        elif self.state == 'ReturnToStart':
            self.stop_robot()
            status_msg = String()
            status_msg.data = 'returning'
            self.status_publisher.publish(status_msg)
            if not self.returned_to_start:
                search_complete_msg = Bool()
                search_complete_msg.data = True
                self.search_complete_pub.publish(search_complete_msg)
                self.get_logger().info('Search Complete: Detected 6 Markers. Returning To Initial State.')
                # (removed duplicate x, y, q, yaw extraction - now using values from above)
                goal_x = self.init_x
                goal_y = self.init_y
                pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                dx = goal_x - x
                dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    self.get_logger().info('Returned to Initial Position. Realigning to Initial Yaw.')
                    if abs(self.angle_diff(self.init_yaw, yaw)) <= 0.5:
                        self.stop_robot()
                        self.get_logger().info('Returned to Initial Orientation. Search Mission Complete.')
                        return_msg = String()
                        return_msg.data = 'arrived to origin'
                        self.report_publisher.publish(return_msg)
                        self.returned_to_start = True
                        search_ended_msg = Bool()
                        search_ended_msg.data = True
                        self.search_ended_pub.publish(search_ended_msg)
                    else:
                        cmd = self.p_orient(x, y, yaw, self.init_x + math.cos(self.init_yaw), self.init_y + math.sin(self.init_yaw))
                        self.publisher_.publish(cmd)
                else:
                    cmd.linear.x = pid_navigate_cmd.linear.x
                    cmd.angular.z = pid_navigate_cmd.angular.z
                    self.publisher_.publish(cmd)
        elif self.state == 'feature_counting':
            self._logger.info('=== In feature_counting state ===(bonus)')
            self.stop_robot()
            bonus_msg=Bool()
            bonus_msg.data=True
            self.bonus_publisher.publish(bonus_msg)
            

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()