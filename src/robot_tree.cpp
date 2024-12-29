// Copyright 2024 Stanislav Mikhel

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include "ext_observer_ros/robot_tree.hpp"


RobotTree::RobotTree()
: RobotDynamicsRnea() {}

std::string RobotTree::init(const std::string& urdf_name)
{
  if (solver_) {
    return "already initialized";
  }

  // Read URDF
  urdf::Model model;
  if (!model.initFile(urdf_name)) {
    return "failed to parse URDF file";
  }

  // to Orocos tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    return "failed to make kinematic tree";
  }

  init_internal(tree);

  return "done";
}

void RobotTree::init_internal(const KDL::Tree& tree)
{
  auto segments = tree.getSegments();
  for (auto it = segments.begin(); it != segments.end(); it++) {
    auto tp = GetTreeElementSegment(it->second).getJoint().getType();
    if (tp != KDL::Joint::Fixed) {
      std::string jnt = GetTreeElementSegment(it->second).getJoint().getName();
      size_t val = GetTreeElementQNr(it->second);
      jnt_map_[jnt] = val;
      joints_.push_back(val);
      names_.push_back(jnt);
    }
    // assume no wrenches
    wmap_[it->first] = KDL::Wrench::Zero();
  }
  joint_tot_ = tree.getNrOfJoints();

  q_.resize(joint_tot_);
  qd_.resize(joint_tot_);
  q2d_.resize(joint_tot_);
  tau_.resize(joint_tot_);

  tau_out_.resize(joints_.size());
  tau_out_.setZero();
  friction_.resize(joints_.size());
  friction_.setZero();

  solver_ = std::make_unique<KDL::TreeIdSolver_RNE>(tree, KDL::Vector(0, 0, GRAVITY));
  solver_no_grav_ = std::make_unique<KDL::TreeIdSolver_RNE>(tree, KDL::Vector::Zero());
}

Vec RobotTree::getFriction(Vec& dq)
{
  // coefficient-wise multiplication
  return friction_.cwiseProduct(dq);
}

void RobotTree::set_friction(const std::vector<std::string>& nms, const std::vector<double>& vs)
{
  fill(nms, vs, friction_);
}

void RobotTree::fill(const std::vector<std::string>& nms, const std::vector<double>& vs, Vec& vec)
{
  vec.resize(joints_.size());
  for (size_t i = 0; i < nms.size(); i++) {
    if (i >= vs.size()) {
      break;
    }
    auto it = jnt_map_.find(nms[i]);
    if (it != jnt_map_.end()) {
      vec(it->second) = vs[i];
    }
  }
}

void RobotTree::fill_inv(
  const Vec& vec, std::vector<std::string>& nms, std::vector<double>& vs)
{
  nms = names_;
  size_t len = joints_.size();
  vs.resize(len);
  for (size_t i = 0; i < len; i++) {
    vs[i] = vec(i);
  }
}

Vec RobotTree::rnea(Vec& q, Vec& qd, Vec& q2d, double g)
{
  if (!solver_) {
    return tau_out_;
  }

  // input -> internal representation
  for (size_t src = 0; src < joints_.size(); ++src) {
    size_t dst = joints_[src];
    q_(dst) = q(src);
    qd_(dst) = qd(src);
    q2d_(dst) = q2d(src);
  }

  // RNEA
  if (g == 0) {
    solver_no_grav_->CartToJnt(q_, qd_, q2d_, wmap_, tau_);
  } else {
    solver_->CartToJnt(q_, qd_, q2d_, wmap_, tau_);
  }

  // internal representation -> output
  for (size_t dst = 0; dst < joints_.size(); ++dst) {
    tau_out_(dst) = tau_(joints_[dst]);
  }

  return tau_out_;
}
