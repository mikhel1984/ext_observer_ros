
// http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
#include <urdf/model.h>

#include <eigen3/Eigen/Dense>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "ext_observer/external_observer.hpp"

class RobotTree : public RobotDynamicsRnea
{
public:
  RobotTree();

  std::string init(const std::string& urdf_name, const std::string& tip);

  Vec rnea(Vec& q, Vec& qd, Vec& q2d, double g=0) override;

  inline int jointNo() override { return joint_no_; }

  Vec getFriction(Vec& qd) override;

private:
  std::unique_ptr<KDL::TreeIdSolver_RNE> solver_;
  std::unique_ptr<KDL::TreeIdSolver_RNE> solver_no_grav_;
  std::unordered_map<std::string, size_t> joints_;
  std::vector<double> friction_;
  KDL::Vector grav_;
  KDL::WrenchMap wmap_;

  unsigned joint_no_ = 0;

};  // RobotKdl

RobotTree::RobotTree()
: RobotDynamicsRnea()
{
  grav_ = KDL::Vector(0, 0, GRAVITY);
}

std::string RobotTree::init(const std::string& urdf_name, const std::string& tip)
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

  auto segments = tree.getSegments();
  for (auto it = segments.begin(); it != segments.end(); it++) {
    joints_[it->first] = GetTreeElementQNr(it->second);
    // assume no wrenches
    wmap_[it->first] = KDL::Wrench::Zero();
  }
  joint_no_ = tree.getNrOfJoints();

  solver_ = std::make_unique<KDL::TreeIdSolver_RNE>(tree, grav_);
  solver_no_grav_ = std::make_unique<KDL::TreeIdSolver_RNE>(tree, KDL::Vector::Zero());

  return "done";
}

Vec RobotTree::getFriction(Vec& dq)
{
  Vec res = -dq;
  for (size_t i = 0; i < friction_.size(); i++) {
    res(i) *= friction_[i];
  }
  return res;
}

Vec RobotTree::rnea(Vec& q, Vec& qd, Vec& q2d, double g)
{
  KDL::JntArray res;
  KDL::JntArray q_0, q_1, q_2;
  q_0.data = q;
  q_1.data = qd;
  q_2.data = q2d;

  if (g == 0) {
    solver_no_grav_->CartToJnt(q_0, q_1, q_2, wmap_, res);
  } else {
    solver_->CartToJnt(q_0, q_1, q_2, wmap_, res);
  }

  return res.data;
}