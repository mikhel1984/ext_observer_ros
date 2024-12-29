// Copyright 2024 Stanislav Mikhel

#ifndef EXT_OBSERVER_ROS__ROBOT_TREE_HPP_
#define EXT_OBSERVER_ROS__ROBOT_TREE_HPP_

#include <eigen3/Eigen/Dense>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>

#include "external_observer.hpp"

/** Robot dynamics implementation. */
class RobotTree final : public RobotDynamicsRnea
{
public:
  RobotTree();

  /**
   * Initialize robot dynamics.
   *
   * @param urdf_name Path to URDF file with robot description.
   * @return 'done' in case of success or error message.
   */
  std::string init(const std::string& urdf_name);

  /**
   * Set (positive) friction coefficients. Friction is assumed proportional to joint velocity.
   *
   * @param nms List of joint names according the URDF file.
   * @param vs List of corresponding friction coefficients.
   */
  void set_friction(const std::vector<std::string>& nms, const std::vector<double>& vs);

  /**
   * Fill Eigen vector in correct order based on joint names.
   *
   * @param nms List of joint names according the URDF file.
   * @param vs List of corresponding values.
   * @param vec Eigen vector to store the result.
   */
  void fill(const std::vector<std::string>& nms, const std::vector<double>& vs, Vec& vec);

  /**
   * Fill list of names and values based on the given vector.
   *
   * @param vec Soruce vector.
   * @param nms List of joint names according the URDF file.
   * @param vs List of corresponding values.
   */
  void fill_inv(const Vec& vec, std::vector<std::string>& nms, std::vector<double>& vs);

  /**
   * Implementation of RNEA algorithm.
   *
   * @param q Vector of joint positions.
   * @param qd Vector of joint velocities.
   * @param q2d Vector of joint torques.
   * @param g Gravity constant.
   * @return vector of joint accelerations.
   */
  Vec rnea(Vec& q, Vec& qd, Vec& q2d, double g = 0) override;

  /**
   * Get number of not-fixed joints.
   *
   * @return movable joint number.
   */
  inline int jointNo() override { return joint_no_; }

  /**
   * Get estimation of friction force.
   *
   * @param qd Vector of joint velocities.
   * @return vector of friction forces.
   */
  Vec getFriction(Vec& qd) override;

private:
  /**
   * Class variables initialization.
   *
   * @param tree Robot kinematic tree structure.
   */
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
  size_t joint_no_ = 0;
};  // RobotTree

#endif  // EXT_OBSERVER_ROS__ROBOT_TREE_HPP_
