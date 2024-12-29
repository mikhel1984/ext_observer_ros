// Copyright 2024 Stanislav Mikhel

#ifndef EXT_OBSERVER_ROS__EXT_OBSERVER_NODE_HPP_
#define EXT_OBSERVER_ROS__EXT_OBSERVER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ext_observer_ros/robot_tree.hpp"

class ExtObserverNode : public rclcpp::Node
{
public:
  ExtObserverNode();

private:
  void estimate(const sensor_msgs::msg::JointState& msg);

  void reset_state(
    const std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr resp);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_torque_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
  std::unique_ptr<ExternalObserverRnea> observer_;
  RobotTree dynamics_;

  Vec q_, dq_, tau_;
  double last_sec_ = -1;
};

#endif  // EXT_OBSERVER_ROS__EXT_OBSERVER_NODE_HPP_
