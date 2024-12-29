#ifndef ROBOT_TREE_HPP
#define ROBOT_TREE_HPP

#include <eigen3/Eigen/Dense>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>

#include "external_observer.hpp"

class RobotTree final : public RobotDynamicsRnea
{
public:
  RobotTree();

  std::string init(const std::string& urdf_name);

  void set_friction(const std::vector<std::string>& nms, const std::vector<double>& vs);

  void fill(const std::vector<std::string>& nms, const std::vector<double>& vs, Vec& vec);

  void fill_inv(const Vec& vec, std::vector<std::string>& nms, std::vector<double>& vs);

  Vec rnea(Vec& q, Vec& qd, Vec& q2d, double g=0) override;

  inline int jointNo() override { return joints_.size(); }

  Vec getFriction(Vec& qd) override;

private:
  void init_internal(const KDL::Tree& tree);

  std::unique_ptr<KDL::TreeIdSolver_RNE> solver_;
  std::unique_ptr<KDL::TreeIdSolver_RNE> solver_no_grav_;
  std::vector<size_t> joints_;
  std::vector<std::string> names_;
  std::unordered_map<std::string, size_t> jnt_map_;
  Vec friction_;
  KDL::WrenchMap wmap_;
  KDL::JntArray q_, qd_, q2d_, tau_;
  Vec tau_out_;

  unsigned joint_tot_ = 0;

};  // RobotTree

#endif  // ROBOT_TREE_HPP