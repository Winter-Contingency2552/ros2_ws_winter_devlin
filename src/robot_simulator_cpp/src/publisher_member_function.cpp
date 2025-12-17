#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/srv/reset_position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Odometry_Node : public rclcpp::Node
{
public:
  Odometry_Node()
  : Node("odometry_node"),
    x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vth_(0.0),
    first_update_(true)
  {
    publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Odometry_Node::topic_callback_cmd_vel, this, _1));

    reset_position_service_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition",
      std::bind(&Odometry_Node::handle_reset_position, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Odometry node initialized and subscribing to /cmd_vel.");

    // ROS-time timer creation (works even if Node::create_timer isn't available)
    timer_ = rclcpp::create_timer(
      this,                           // node base interface
      this->get_clock(),              // IMPORTANT: ROS clock (respects /clock when use_sim_time=true)
      std::chrono::milliseconds(50),
      std::bind(&Odometry_Node::tf_update, this));
  }

private:
  void tf_update()
  {
    auto now = this->get_clock()->now();

    if (first_update_) {
      last_time_ = now;
      first_update_ = false;
      return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Integrate
    x_ += vx_ * dt * std::cos(th_);
    y_ += vx_ * dt * std::sin(th_);
    th_ += vth_ * dt;

    // Normalize yaw
    th_ = std::atan2(std::sin(th_), std::cos(th_));

    // TF: world -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    t.transform.rotation = tf2::toMsg(q);

    publisher_->sendTransform(t);

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = t.transform.rotation;
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = vth_;
    publisher_odom_->publish(odom);

    // Path
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now;
    p.header.frame_id = "world";
    p.pose = odom.pose.pose;

    path_msg_.header.stamp = now;
    path_msg_.header.frame_id = "world";
    path_msg_.poses.push_back(p);
    publisher_path_->publish(path_msg_);
  }

  void topic_callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg_cmd)
  {
    vx_ = msg_cmd->linear.x;
    vth_ = msg_cmd->angular.z;
  }

  void handle_reset_position(
    const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> request,
    std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> response)
  {
    x_ = request->pose.position.x;
    y_ = request->pose.position.y;

    const auto & q = request->pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    th_ = std::atan2(siny_cosp, cosy_cosp);

    first_update_ = true;  // re-init timing on next tick
    response->success = true;
  }

  // --- Members (order matters for -Wreorder warnings) ---
  std::shared_ptr<tf2_ros::TransformBroadcaster> publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
  nav_msgs::msg::Path path_msg_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_position_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  double x_;
  double y_;
  double th_;
  double vx_;
  double vth_;
  bool first_update_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry_Node>());
  rclcpp::shutdown();
  return 0;
}
