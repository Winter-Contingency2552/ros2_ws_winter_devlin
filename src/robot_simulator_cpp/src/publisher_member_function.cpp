// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/srv/reset_position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Odometry_Node : public rclcpp::Node
{
public:
  Odometry_Node()
  : Node("odometry_node"), x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vth_(0.0)
  {
    // TF publisher
    publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Odom and Path publishers
    publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    // Cmd_vel subscriber
    subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&Odometry_Node::topic_callback_cmd_vel, this, _1));

    // Reset position service
    reset_position_service_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition", std::bind(&Odometry_Node::handle_reset_position, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Odometry node initialized and subscribing to /cmd_vel.");
    last_time_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Odometry_Node::tf_update, this));
  }
  

private:
  void tf_update()
  {
    auto message = tf2_msgs::msg::TFMessage();
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Update the robot's position based on velocity and time elapsed
    double delta_x = vx_ * dt * std::cos(th_);
    double delta_y = vx_ * dt * std::sin(th_);
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Build one TransformStamped message
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now; // Required
    t.header.frame_id = "odom"; // Parent frame
    t.child_frame_id = "base_link"; // Child frame

    // Translation Structure
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    // Rotation Structure (Quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    t.transform.rotation = tf2::toMsg(q);

    // Publish Odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    auto odom_quat = tf2::Quaternion();
    odom_quat.setRPY(0, 0, th_);
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = vth_;
    publisher_odom_->publish(odom);

    // Publish Path
    auto p = geometry_msgs::msg::PoseStamped();
    p.header.stamp = now;
    p.header.frame_id = "odom";
    p.pose = odom.pose.pose;
    path_msg_.header.stamp = now;
    path_msg_.header.frame_id = "odom";
    path_msg_.poses.push_back(p);
    publisher_path_->publish(path_msg_);

    // Publish the TransformStamped
    publisher_->sendTransform(t);

    // Print out the tf for debugging (10 Hz)
    // RCLCPP_INFO(this->get_logger(), "Publishing Transform: x: '%.2f', y: '%.2f', th: '%.2f'", x_, y_, th_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
  nav_msgs::msg::Path path_msg_;
  size_t count_;

  void topic_callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg_cmd)
  {
    vx_ = msg_cmd->linear.x;
    vth_ = msg_cmd->angular.z;
    // RCLCPP_INFO(this->get_logger(), "I heard: linear x: '%.2f', angular z: '%.2f'", msg_cmd->linear.x, msg_cmd->angular.z);
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

    // RCLCPP_INFO(this->get_logger(), "Position reset to x: '%.2f', y: '%.2f', th: '%.2f'", x_, y_, th_);
    response->success = true;
  }
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_position_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
  rclcpp::Time last_time_;

  double x_;
  double y_;
  double th_;
  double vx_;
  double vth_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry_Node>());
  rclcpp::shutdown();
  return 0;
}
