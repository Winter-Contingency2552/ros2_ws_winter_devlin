import cmd
from itertools import accumulate
from re import search
from textwrap import wrap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from aruco_interfaces.msg import ArucoMarkerArray
from std_msgs.msg import Bool

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.search_complete_pub = self.create_publisher(Bool, 'search_complete', 10)
        self.search_ended_pub = self.create_publisher(Bool, 'search_ended', 10)

        self.sub = self.create_subscription(ArucoMarkerArray, "/aruco/markers", self.marker_callback, 10)

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
        self.Num_markers_detected = len(msg.markers)
        self.get_logger().info(
            f"Got {self.Num_markers_detected} markers in frame '{msg.header.frame_id}' at stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

    def control_loop(self):
        try:
            tf = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF lookup failed. Retrying...')
            return
        if self.Num_markers_detected < 6:
            search_complete_msg = Bool()
            search_complete_msg.data = False
            search_ended_msg = Bool()
            search_ended_msg.data = False
            self.search_ended_pub.publish(search_ended_msg)
            self.search_complete_pub.publish(search_complete_msg)
            # Extract TF Values
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = self.yaw_from_quaternion(q)

            cmd = Twist()

            # State Machine Logic
            if self.state == 'Initial':
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
                quarter_x = (self.checkpoint_x + midpoint_x) / 2
                quarter_y = (self.checkpoint_y + midpoint_y) / 2
                three_quarter_x = (midpoint_x + goal_x) / 2
                three_quarter_y = (midpoint_y + goal_y) / 2
                if self.side_index == 1 or self.side_index == 3:  # For short sides, check only at midpoint and goal point
                    if self.wall_checked_1 is False:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, midpoint_x, midpoint_y)
                        dx = midpoint_x - x
                        dy = midpoint_y - y
                    else:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                        dx = goal_x - x
                        dy = goal_y - y
                else:  # For long sides, check at quarter, midpoint, three-quarter, and goal point
                    if self.wall_checked_1 is False:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, quarter_x, quarter_y)
                        dx = quarter_x - x
                        dy = quarter_y - y
                    elif self.wall_checked_2 is False:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, midpoint_x, midpoint_y)
                        dx = midpoint_x - x
                        dy = midpoint_y - y
                    elif self.wall_checked_3 is False:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, three_quarter_x, three_quarter_y)
                        dx = three_quarter_x - x
                        dy = three_quarter_y - y
                    else:
                        pid_navigate_cmd = self.pid_navigate(x, y, yaw, goal_x, goal_y)
                        dx = goal_x - x
                        dy = goal_y - y
                distance_to_goal = math.sqrt(dx**2 + dy**2)
                if distance_to_goal <= self.pos_tol:
                    self.stop_robot()
                    if self.side_index == 1 or self.side_index == 3:  # Short sides
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
                    else:  # Long sides
                        if self.wall_checked_1 is False:
                            self.wall_checked_1 = True
                            self.get_logger().info('Reached Quarter Point At ({:.2f}, {:.2f}), preparing to check wall.'.format(quarter_x, quarter_y))
                            self.state = 'WallCheck'
                            self.get_logger().info('State changed to WallCheck')
                        elif self.wall_checked_2 is False:
                            self.wall_checked_2 = True
                            self.get_logger().info('Reached Midpoint At ({:.2f}, {:.2f}), preparing to check wall.'.format(midpoint_x, midpoint_y))
                            self.state = 'WallCheck'
                            self.get_logger().info('State changed to WallCheck')
                        elif self.wall_checked_3 is False:
                            self.wall_checked_3 = True
                            self.get_logger().info('Reached Three-Quarter Point At ({:.2f}, {:.2f}), preparing to check wall.'.format(three_quarter_x, three_quarter_y))
                            self.state = 'WallCheck'
                            self.get_logger().info('State changed to WallCheck')
                        elif self.wall_checked_4 is False:
                            self.wall_checked_4 = True
                            self.get_logger().info('Reached Goal Point At ({:.2f}, {:.2f}), preparing to check wall.'.format(goal_x, goal_y))
                            self.state = 'WallCheck'
                            self.get_logger().info('State changed to WallCheck')
                        else:
                            self.state = 'OrientCorner'
                            self.wall_checked_1 = False
                            self.wall_checked_2 = False
                            self.wall_checked_3 = False
                            self.wall_checked_4 = False
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
        else:
            self.stop_robot()
            if not self.returned_to_start:
                search_complete_msg = Bool()
                search_complete_msg.data = True
                self.search_complete_pub.publish(search_complete_msg)
                self.get_logger().info('Search Complete: Detected 6 Markers. Returning To Initial State.')
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                q = tf.transform.rotation
                yaw = self.yaw_from_quaternion(q)
                cmd = Twist()
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

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()