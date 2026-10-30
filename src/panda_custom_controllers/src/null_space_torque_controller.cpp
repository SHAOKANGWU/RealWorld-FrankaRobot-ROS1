#include <panda_custom_controllers/null_space_torque_controller.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <franka/model.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_custom_controllers {

namespace {
constexpr std::size_t kNumberOfJoints = 7;

bool getJointGains(ros::NodeHandle& node_handle,
                   const std::string& parameter_name,
                   std::array<double, kNumberOfJoints>& gains) {
  std::vector<double> values;
  if (!node_handle.getParam(parameter_name, values) || values.size() != kNumberOfJoints) {
    ROS_ERROR_STREAM("NullSpaceTorqueController: parameter '" << parameter_name
                     << "' must contain exactly seven values.");
    return false;
  }

  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(values[i]) || values[i] < 0.0) {
      ROS_ERROR_STREAM("NullSpaceTorqueController: parameter '" << parameter_name
                       << "' must contain finite, non-negative values.");
      return false;
    }
    gains[i] = values[i];
  }
  return true;
}
}  // namespace

constexpr std::size_t NullSpaceTorqueController::kNumberOfJoints;

bool NullSpaceTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                     ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("NullSpaceTorqueController: missing required parameter 'arm_id'.");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) ||
      joint_names.size() != kNumberOfJoints) {
    ROS_ERROR("NullSpaceTorqueController: 'joint_names' must contain seven joints.");
    return false;
  }

  if (!getJointGains(node_handle, "k_gains", k_gains_) ||
      !getJointGains(node_handle, "d_gains", d_gains_)) {
    return false;
  }

  node_handle.param<std::string>("base_frame", base_frame_, arm_id + "_link0");
  node_handle.param("nullspace_weight", nullspace_weight_, 1.0);
  node_handle.param("pseudoinverse_tolerance", pseudoinverse_tolerance_, 1e-6);
  node_handle.param("coriolis_factor", coriolis_factor_, 1.0);
  node_handle.param("delta_tau_max", delta_tau_max_, 1.0);

  if (base_frame_.empty() || !std::isfinite(nullspace_weight_) || nullspace_weight_ < 0.0 ||
      !std::isfinite(pseudoinverse_tolerance_) || pseudoinverse_tolerance_ <= 0.0 ||
      !std::isfinite(coriolis_factor_) || !std::isfinite(delta_tau_max_) ||
      delta_tau_max_ <= 0.0) {
    ROS_ERROR("NullSpaceTorqueController: invalid controller configuration.");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  auto* effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (model_interface == nullptr || state_interface == nullptr || effort_interface == nullptr) {
    ROS_ERROR("NullSpaceTorqueController: required hardware interfaces are unavailable.");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));

    joint_handles_.reserve(kNumberOfJoints);
    for (const auto& joint_name : joint_names) {
      joint_handles_.push_back(effort_interface->getHandle(joint_name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR_STREAM("NullSpaceTorqueController: failed to get a hardware handle: "
                     << exception.what());
    return false;
  }

  std::string command_topic;
  node_handle.param<std::string>(
      "command_topic", command_topic, "desired_null_space_command");
  command_subscriber_ = node_handle.subscribe(
      command_topic,
      1,
      &NullSpaceTorqueController::commandCallback,
      this,
      ros::TransportHints().tcpNoDelay());

  ROS_INFO_STREAM("NullSpaceTorqueController: listening for synchronized pose and joint "
                     "commands with gripper state in frame '"
                  << base_frame_ << "' on '" << node_handle.resolveName(command_topic) << "'.");

  node_handle.param("enable_gripper", gripper_enabled_, true);
  if (gripper_enabled_) {
    std::string gripper_move_action_name;
    std::string gripper_grasp_action_name;
    node_handle.param<std::string>(
        "gripper_move_action_name", gripper_move_action_name, "/franka_gripper/move");
    node_handle.param<std::string>(
        "gripper_grasp_action_name", gripper_grasp_action_name, "/franka_gripper/grasp");
    node_handle.param("gripper_speed", gripper_speed_, 0.05);
    node_handle.param("gripper_force", gripper_force_, 20.0);
    node_handle.param("gripper_open_width", gripper_open_width_, 0.08);
    node_handle.param("gripper_grasp_width", gripper_grasp_width_, 0.0);
    node_handle.param("gripper_epsilon_inner", gripper_epsilon_inner_, 0.005);
    node_handle.param("gripper_epsilon_outer", gripper_epsilon_outer_, 0.08);
    node_handle.param("gripper_server_timeout", gripper_server_timeout_, 0.5);

    if (!std::isfinite(gripper_speed_) || gripper_speed_ <= 0.0 ||
        !std::isfinite(gripper_force_) || gripper_force_ <= 0.0 ||
        !std::isfinite(gripper_open_width_) || gripper_open_width_ < 0.0 ||
        !std::isfinite(gripper_grasp_width_) || gripper_grasp_width_ < 0.0 ||
        gripper_grasp_width_ > gripper_open_width_ ||
        !std::isfinite(gripper_epsilon_inner_) || gripper_epsilon_inner_ < 0.0 ||
        !std::isfinite(gripper_epsilon_outer_) || gripper_epsilon_outer_ < 0.0 ||
        !std::isfinite(gripper_server_timeout_) || gripper_server_timeout_ < 0.0) {
      ROS_ERROR("NullSpaceTorqueController: invalid gripper configuration.");
      return false;
    }

    gripper_move_client_ =
        std::make_unique<GripperMoveClient>(node_handle, gripper_move_action_name, true);
    gripper_grasp_client_ =
        std::make_unique<GripperGraspClient>(node_handle, gripper_grasp_action_name, true);
  }
  return true;
}

void NullSpaceTorqueController::starting(const ros::Time& /*time*/) {
  const auto robot_state = state_handle_->getRobotState();
  const Eigen::Map<const Eigen::Matrix<double, 4, 4>> transform(robot_state.O_T_EE.data());
  const Eigen::Quaterniond orientation(transform.block<3, 3>(0, 0));

  ControllerCommand initial_command;
  initial_command.pose.position = {{transform(0, 3), transform(1, 3), transform(2, 3)}};
  initial_command.pose.orientation =
      {{orientation.x(), orientation.y(), orientation.z(), orientation.w()}};
  initial_command.joint_positions = robot_state.q;
  desired_command_buffer_.writeFromNonRT(initial_command);
  received_gripper_command_ = false;
}

void NullSpaceTorqueController::update(const ros::Time& /*time*/,
                                       const ros::Duration& /*period*/) {
  const auto robot_state = state_handle_->getRobotState();
  const auto coriolis_array = model_handle_->getCoriolis();
  const auto jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const ControllerCommand command = *desired_command_buffer_.readFromRT();

  const Eigen::Map<const Vector7d> q(robot_state.q.data());
  const Eigen::Map<const Vector7d> dq(robot_state.dq.data());
  const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
  const Eigen::Map<const Jacobian> jacobian(jacobian_array.data());
  const Eigen::Map<const Eigen::Matrix<double, 4, 4>> transform(robot_state.O_T_EE.data());
  const Eigen::Map<const Vector7d> desired_joint_positions(command.joint_positions.data());

  const Eigen::Vector3d current_position = transform.block<3, 1>(0, 3);
  Eigen::Quaterniond current_orientation(transform.block<3, 3>(0, 0));
  const Eigen::Vector3d desired_position(
      command.pose.position[0], command.pose.position[1], command.pose.position[2]);
  const Eigen::Quaterniond desired_orientation(command.pose.orientation[3],
                                               command.pose.orientation[0],
                                               command.pose.orientation[1],
                                               command.pose.orientation[2]);

  // Select the equivalent current quaternion that gives the shortest orientation error.
  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() *= -1.0;
  }

  Vector6d pose_error;
  pose_error.head<3>() = desired_position - current_position;
  const Eigen::Quaterniond orientation_error =
      desired_orientation.inverse() * current_orientation;
  pose_error.tail<3>() << orientation_error.x(), orientation_error.y(), orientation_error.z();
  pose_error.tail<3>() = -transform.block<3, 3>(0, 0) * pose_error.tail<3>();

  const Eigen::JacobiSVD<Jacobian> svd(
      jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  JacobianPseudoInverse inverse_singular_values = JacobianPseudoInverse::Zero();
  const auto singular_values = svd.singularValues();
  const double singular_value_threshold =
      pseudoinverse_tolerance_ * std::max(1.0, singular_values[0]);
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values[i] > singular_value_threshold) {
      inverse_singular_values(i, i) = 1.0 / singular_values[i];
    }
  }
  const JacobianPseudoInverse jacobian_pseudoinverse =
      svd.matrixV() * inverse_singular_values * svd.matrixU().transpose();
  const Matrix7d nullspace_projector =
      Matrix7d::Identity() - jacobian_pseudoinverse * jacobian;

  const Vector7d primary_joint_motion = jacobian_pseudoinverse * pose_error;
  const Vector7d secondary_joint_motion =
      nullspace_weight_ * nullspace_projector * (desired_joint_positions - q);
  const Vector7d unified_joint_error = primary_joint_motion + secondary_joint_motion;

  JointArray desired_torques{};
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    const auto index = static_cast<Eigen::Index>(i);
    desired_torques[i] = k_gains_[i] * unified_joint_error[index] -
                         d_gains_[i] * dq[index] + coriolis_factor_ * coriolis[index];
  }

  const auto saturated_torques = saturateTorqueRate(desired_torques, robot_state.tau_J_d);
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    joint_handles_[i].setCommand(saturated_torques[i]);
  }
}

void NullSpaceTorqueController::commandCallback(const NullSpaceCommandConstPtr& message) {
  if (!message->header.frame_id.empty() && message->header.frame_id != base_frame_) {
    ROS_ERROR_THROTTLE(1.0,
                       "NullSpaceTorqueController: pose frame must be '%s' (received '%s').",
                       base_frame_.c_str(),
                       message->header.frame_id.c_str());
    return;
  }

  const auto& position = message->pose.position;
  const auto& orientation = message->pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w)) {
    ROS_ERROR_THROTTLE(1.0, "NullSpaceTorqueController: rejecting a non-finite pose.");
    return;
  }

  Eigen::Quaterniond normalized_orientation(
      orientation.w, orientation.x, orientation.y, orientation.z);
  if (normalized_orientation.norm() < 1e-6) {
    ROS_ERROR_THROTTLE(1.0, "NullSpaceTorqueController: rejecting a zero quaternion.");
    return;
  }
  normalized_orientation.normalize();

  ControllerCommand command;
  command.pose.position = {{position.x, position.y, position.z}};
  command.pose.orientation = {{normalized_orientation.x(),
                               normalized_orientation.y(),
                               normalized_orientation.z(),
                               normalized_orientation.w()}};
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(message->joint_positions[i])) {
      ROS_ERROR_THROTTLE(
          1.0, "NullSpaceTorqueController: rejecting non-finite joint positions.");
      return;
    }
    command.joint_positions[i] = message->joint_positions[i];
  }
  desired_command_buffer_.writeFromNonRT(command);
  handleGripperCommand(message->gripper_grasp);
}

void NullSpaceTorqueController::handleGripperCommand(bool grasp) {
  if (!gripper_enabled_) {
    return;
  }
  if (received_gripper_command_ && grasp == last_gripper_command_) {
    return;
  }

  if (grasp) {
    if (!gripper_grasp_client_->isServerConnected() &&
        !gripper_grasp_client_->waitForServer(ros::Duration(gripper_server_timeout_))) {
      ROS_WARN_THROTTLE(
          1.0, "NullSpaceTorqueController: Franka gripper grasp action is unavailable.");
      return;
    }

    gripper_move_client_->cancelAllGoals();
    franka_gripper::GraspGoal goal;
    goal.width = gripper_grasp_width_;
    goal.epsilon.inner = gripper_epsilon_inner_;
    goal.epsilon.outer = gripper_epsilon_outer_;
    goal.speed = gripper_speed_;
    goal.force = gripper_force_;
    gripper_grasp_client_->sendGoal(goal);
  } else {
    if (!gripper_move_client_->isServerConnected() &&
        !gripper_move_client_->waitForServer(ros::Duration(gripper_server_timeout_))) {
      ROS_WARN_THROTTLE(
          1.0, "NullSpaceTorqueController: Franka gripper move action is unavailable.");
      return;
    }

    gripper_grasp_client_->cancelAllGoals();
    franka_gripper::MoveGoal goal;
    goal.width = gripper_open_width_;
    goal.speed = gripper_speed_;
    gripper_move_client_->sendGoal(goal);
  }

  received_gripper_command_ = true;
  last_gripper_command_ = grasp;
}

NullSpaceTorqueController::JointArray NullSpaceTorqueController::saturateTorqueRate(
    const JointArray& desired_torques,
    const JointArray& previous_desired_torques) const {
  JointArray saturated_torques{};
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    const double difference = desired_torques[i] - previous_desired_torques[i];
    saturated_torques[i] = previous_desired_torques[i] +
                           std::max(-delta_tau_max_, std::min(delta_tau_max_, difference));
  }
  return saturated_torques;
}

}  // namespace panda_custom_controllers

PLUGINLIB_EXPORT_CLASS(panda_custom_controllers::NullSpaceTorqueController,
                       controller_interface::ControllerBase)
