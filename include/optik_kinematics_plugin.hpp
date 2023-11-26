#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Dense>
#include <optik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace optik_kinematics_plugin {

class OptIkKinematicsPlugin : public kinematics::KinematicsBase {
 public:
  bool initialize(const rclcpp::Node::SharedPtr &node,
                  const moveit::core::RobotModel &robot_model,
                  const std::string &group_name, const std::string &base_frame,
                  const std::vector<std::string> &tip_frames,
                  double search_discretization) override;

  const std::vector<std::string> &getJointNames() const override;

  const std::vector<std::string> &getLinkNames() const override;

  bool getPositionFK(const std::vector<std::string> &,
                     const std::vector<double> &,
                     std::vector<geometry_msgs::msg::Pose> &) const override;

  bool getPositionIK(
      const geometry_msgs::msg::Pose &ik_pose,
      const std::vector<double> &ik_seed_state, std::vector<double> &solution,
      moveit_msgs::msg::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose &ik_pose,
      const std::vector<double> &ik_seed_state, double timeout,
      std::vector<double> &solution,
      moveit_msgs::msg::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose &ik_pose,
      const std::vector<double> &ik_seed_state, double timeout,
      const std::vector<double> &consistency_limits,
      std::vector<double> &solution,
      moveit_msgs::msg::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose &ik_pose,
      const std::vector<double> &ik_seed_state, double timeout,
      std::vector<double> &solution, const IKCallbackFn &solution_callback,
      moveit_msgs::msg::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose &ik_pose,
      const std::vector<double> &ik_seed_state, double timeout,
      const std::vector<double> &consistency_limits,
      std::vector<double> &solution, const IKCallbackFn &solution_callback,
      moveit_msgs::msg::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options =
          kinematics::KinematicsQueryOptions()) const override;

 private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  std::shared_ptr<optik::Robot> robot_;
  optik::SolverConfig solver_config_;
};

}  // namespace optik_kinematics_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(optik_kinematics_plugin::OptIkKinematicsPlugin,
                       kinematics::KinematicsBase);
