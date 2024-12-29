#ifndef EXT_OBSERVER_NODE_HPP
#define EXT_OBSERVER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ext_observer_ros/robot_tree.hpp"

class ExtObserverNode : public rclcpp::Node
{
public:
  ExtObserverNode();

private:
  void estimate(const sensor_msgs::msg::JointState& msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_torque_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
  std::unique_ptr<ExternalObserverRnea> observer_;
  RobotTree dynamics_;

  Vec q_, dq_, tau_;
  double last_sec_ = -1;
};


#endif  // EXT_OBSERVER_NODE_HPP