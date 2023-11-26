#include "optik_kinematics_plugin.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Dense>
#include <optik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace optik_kinematics_plugin {

bool OptIkKinematicsPlugin::initialize(
    const rclcpp::Node::SharedPtr &node,
    const moveit::core::RobotModel &robot_model, const std::string &group_name,
    const std::string &base_frame, const std::vector<std::string> &tip_frames,
    double search_discretization) {
  storeValues(robot_model, group_name, base_frame, tip_frames,
              search_discretization);

  const auto joint_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_group) {
    RCLCPP_ERROR(LOGGER, "failed to get joint model group %s",
                 group_name.c_str());
    return false;
  }

  // TODO: Hopefully these appear in the same order as in the URDF.
  joint_names_ = joint_group->getJointModelNames();
  link_names_ = {joint_group->getLinkModelNames().at(0)};

  // TODO: This only handles the case when len(tip_frames) == 0.
  // TODO: Maybe this should be translated from the `robot_model` data
  // structure.
  const auto &robot_description =
      node->get_parameter("robot_description").get_value<std::string>();
  robot_ = std::make_shared<optik::Robot>(std::move(optik::Robot::FromUrdfStr(
      robot_description, base_frame, tip_frames.at(0))));

  return true;
}

const std::vector<std::string> &OptIkKinematicsPlugin::getJointNames() const {
  return joint_names_;
}

const std::vector<std::string> &OptIkKinematicsPlugin::getLinkNames() const {
  return link_names_;
}

bool OptIkKinematicsPlugin::getPositionFK(
    const std::vector<std::string> &, const std::vector<double> &,
    std::vector<geometry_msgs::msg::Pose> &) const {
  // TODO
  return false;
}

bool OptIkKinematicsPlugin::getPositionIK(
    const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  std::vector<double> consistency_limits;

  // limit search to a single attempt by setting a timeout of zero
  // TODO: Limiting to zero time is (highly) unlikely to produce a result.
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits,
                          solution, IKCallbackFn(), error_code, options);
}

bool OptIkKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, IKCallbackFn(), error_code, options);
}

bool OptIkKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, IKCallbackFn(), error_code, options);
}

bool OptIkKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution, const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, IKCallbackFn(), error_code, options);
}

bool OptIkKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution, const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  // TODO: Load some of these from ROS parameters (e.g. tol).
  optik::SolverConfig solver_config;
  solver_config.gradient_mode = optik::GradientMode::kAnalytical;
  solver_config.solution_mode = optik::SolutionMode::kSpeed;
  solver_config.tol_f = 1e-15;
  solver_config.max_time = timeout;

  Eigen::Isometry3d target_ee_pose;
  tf2::fromMsg(ik_pose, target_ee_pose);

  const Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(
      ik_seed_state.data(), ik_seed_state.size());

  Eigen::VectorXd q;
  if (robot_->DoIk(solver_config, target_ee_pose, x0, &q)) {
    solution.assign(q.begin(), q.end());
    return true;
  }

  return false;
}

};  // namespace optik_kinematics_plugin
