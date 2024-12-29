// Copyright 2024 Stanislav Mikhel

#include "ext_observer_ros/ext_observer_node.hpp"

#include "disturbance_observer_rnea.hpp"
#include "filtered_dyn_observer_rnea.hpp"
#include "momentum_observer_rnea.hpp"
#include "sliding_mode_observer_rnea.hpp"


ExtObserverNode::ExtObserverNode()
: Node("ext_observer")
{
  // parameters
  std::string urdf_path = this->declare_parameter<std::string>("urdf", "");
  std::string observer_type = this->declare_parameter<std::string>("observer_type", "filter");
  this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());
  this->declare_parameter<std::vector<double>>("friction", std::vector<double>());
  // momentum observer
  this->declare_parameter<std::vector<double>>("momentum_gains", std::vector<double>());
  // disturbance observer
  this->declare_parameter<double>("disturbance_sigma", 21);
  this->declare_parameter<double>("disturbance_xeta", 18);
  this->declare_parameter<double>("disturbance_beta", 50);
  // filtered dynamic
  this->declare_parameter<double>("filter_cutoff", 10);
  this->declare_parameter<double>("filter_sample", 0.01);
  // sliding mode
  this->declare_parameter<std::vector<double>>("sm_s1", std::vector<double>());
  this->declare_parameter<std::vector<double>>("sm_s2", std::vector<double>());

  std::string res = dynamics_.init(urdf_path);
  if (res != "done") {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Robot dynamic initialization error: " << res);
    return;
  }

  if (observer_type == "momentum") {
    Vec gains;
    dynamics_.fill(
      this->get_parameter("joint_names").as_string_array(),
      this->get_parameter("momentum_gains").as_double_array(),
      gains);
    observer_ = std::make_unique<MomentumObserverRnea>(&dynamics_, gains);

  } else if (observer_type == "disturbance") {
    observer_ = std::make_unique<DisturbanceObserverRnea>(
      &dynamics_,
      this->get_parameter("disturbance_sigma").as_double(),
      this->get_parameter("disturbance_xeta").as_double(),
      this->get_parameter("disturbance_beta").as_double());

  } else if (observer_type == "filter") {
    observer_ = std::make_unique<FDynObserverRnea>(
      &dynamics_,
      this->get_parameter("filter_cutoff").as_double(),
      this->get_parameter("filter_sample").as_double());

  } else if (observer_type == "sm") {
    Vec s1, s2, t1, t2;
    dynamics_.fill(
      this->get_parameter("joint_names").as_string_array(),
      this->get_parameter("sm_s1").as_double_array(),
      s1);
    dynamics_.fill(
      this->get_parameter("joint_names").as_string_array(),
      this->get_parameter("sm_s2").as_double_array(),
      s2);
    t1 = 2 * s1.array().sqrt();
    t2 = 2 * s2.array().sqrt();
    observer_ = std::make_unique<SlidingModeObserverRnea>(
      &dynamics_, t1, s1, t2, s2);
  } else {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Unknown observer type " << observer_type);
    return;
  }

  using std::placeholders::_1;
  using std::placeholders::_2;

  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
  pub_torque_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/out/ext_torque", sensor_qos);
  sub_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/in/joint_state", sensor_qos, std::bind(&ExtObserverNode::estimate, this, _1));
  srv_reset_ = this->create_service<std_srvs::srv::Empty>(
    "/ext_observer_reset", std::bind(&ExtObserverNode::reset_state, this, _1, _2));
}

void ExtObserverNode::estimate(const sensor_msgs::msg::JointState& msg)
{
  double sec = msg.header.stamp.sec + msg.header.stamp.nanosec*1E-9;
  if (last_sec_ < 0) {
    // skip first
    last_sec_ = sec;
    return;
  }
  double dt = sec - last_sec_;

  dynamics_.fill(msg.name, msg.position, q_);
  dynamics_.fill(msg.name, msg.velocity, dq_);
  dynamics_.fill(msg.name, msg.effort, tau_);

  Vec res = observer_->getExternalTorque(q_, dq_, tau_, dt);

  sensor_msgs::msg::JointState out;
  out.header = msg.header;  // keep the same time and frame id
  dynamics_.fill_inv(res, out.name, out.effort);

  pub_torque_->publish(out);
}

void ExtObserverNode::reset_state(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr req,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr resp)
{
  observer_->reset();
}


// call
int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtObserverNode>());
  rclcpp::shutdown();
  return 0;
}
