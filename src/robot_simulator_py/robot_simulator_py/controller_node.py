import cmd
from textwrap import wrap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # TF2 buffer and listener reading odom to base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State Machine Variables
        self.state = 'Initial'
        self.side_index = 0 # 0: front, 1: right, 2: back, 3: left
        self.side_length = 2.0 # meters (all sides of desired square)
        self.speed = 0.2 # m/s
        self.angular_speed = 0.1 # rad/s

        self.pos_tol = 0.01 # [m] tolerance for position
        self.ang_tol = math.radians(0.1) # [rad] tolerance for angle

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0

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

    # ---- Primary Loop ---- #

    def control_loop(self):
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF lookup failed. Retrying...')
            return

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
            self.state = 'DriveForward'
            self.get_logger().info('State changed to DriveForward')

        elif self.state == 'DriveForward':
            dx = x - self.init_x
            dy = y - self.init_y
            dist_moved = math.sqrt(dx**2 + dy**2)

            if dist_moved >= self.side_length - self.pos_tol:
                self.stop_robot()
                self.state = 'Turn'
                self.get_logger().info('State changed to Turn')
            else:
                cmd.linear.x = self.speed
                cmd.angular.z = 0.0

        elif self.state == 'Turn':
            target_yaw = self.init_yaw + (math.pi / 2)
            target_yaw = self.wrap_to_pi(target_yaw)

            yaw_diff = self.angle_diff(target_yaw, yaw)

            if abs(yaw_diff) <= self.ang_tol:
                self.stop_robot()
                self.side_index += 1
                if self.side_index >= 4:
                    self.state = 'Finished'
                    self.get_logger().info('State changed to Finished')
                else:
                    self.init_x = x
                    self.init_y = y
                    self.init_yaw = yaw
                    self.state = 'DriveForward'
                    self.get_logger().info('State changed to DriveForward')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if yaw_diff > 0 else -self.angular_speed

        elif self.state == 'Finished':
            self.stop_robot()
            self.get_logger().info('Square path complete. Robot stopped.')

        # Publish command
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()