#include <panda_custom_controllers/joint_position_torque_controller.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_custom_controllers {

namespace {
constexpr std::size_t kNumberOfJoints = 7;

bool getGains(ros::NodeHandle& node_handle,
              const std::string& parameter_name,
              std::array<double, kNumberOfJoints>& gains) {
  std::vector<double> values;
  if (!node_handle.getParam(parameter_name, values) || values.size() != kNumberOfJoints) {
    ROS_ERROR_STREAM("Parameter '" << parameter_name << "' must contain exactly "
                                    << kNumberOfJoints << " values.");
    return false;
  }

  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(values[i]) || values[i] < 0.0) {
      ROS_ERROR_STREAM("Parameter '" << parameter_name
                                      << "' must contain finite, non-negative values.");
      return false;
    }
    gains[i] = values[i];
  }
  return true;
}
}  // namespace

constexpr std::size_t JointPositionTorqueController::kNumberOfJoints;

bool JointPositionTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                         ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointPositionTorqueController: missing required parameter 'arm_id'.");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) ||
      joint_names.size() != kNumberOfJoints) {
    ROS_ERROR("JointPositionTorqueController: 'joint_names' must contain seven joints.");
    return false;
  }

  if (!getGains(node_handle, "k_gains", k_gains_) ||
      !getGains(node_handle, "d_gains", d_gains_)) {
    return false;
  }

  node_handle.param("coriolis_factor", coriolis_factor_, 1.0);
  node_handle.param("delta_tau_max", delta_tau_max_, 1.0);
  if (!std::isfinite(coriolis_factor_) || !std::isfinite(delta_tau_max_) ||
      delta_tau_max_ <= 0.0) {
    ROS_ERROR("JointPositionTorqueController: invalid Coriolis factor or torque-rate limit.");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  auto* effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (model_interface == nullptr || state_interface == nullptr || effort_interface == nullptr) {
    ROS_ERROR("JointPositionTorqueController: required hardware interfaces are unavailable.");
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
    ROS_ERROR_STREAM("JointPositionTorqueController: failed to get a hardware handle: "
                     << exception.what());
    return false;
  }

  std::string command_topic;
  node_handle.param<std::string>(
      "command_topic", command_topic, "desired_joint_position_command");
  command_subscriber_ = node_handle.subscribe(
      command_topic,
      1,
      &JointPositionTorqueController::commandCallback,
      this,
      ros::TransportHints().tcpNoDelay());

  ROS_INFO_STREAM("JointPositionTorqueController: listening for joint-position and gripper "
                     "commands on '"
                  << node_handle.resolveName(command_topic) << "'.");

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
      ROS_ERROR("JointPositionTorqueController: invalid gripper configuration.");
      return false;
    }

    // The action client owns a spinner thread, so waiting for the gripper server in the command
    // callback never blocks the real-time arm update loop.
    gripper_move_client_ =
        std::make_unique<GripperMoveClient>(node_handle, gripper_move_action_name, true);
    gripper_grasp_client_ =
        std::make_unique<GripperGraspClient>(node_handle, gripper_grasp_action_name, true);
  }
  return true;
}

void JointPositionTorqueController::starting(const ros::Time& /*time*/) {
  // Hold the measured pose until the first valid command arrives. This avoids a jump when the
  // controller is switched on or restarted.
  desired_positions_buffer_.writeFromNonRT(state_handle_->getRobotState().q);
  received_gripper_command_ = false;
}

void JointPositionTorqueController::update(const ros::Time& /*time*/,
                                           const ros::Duration& /*period*/) {
  const auto robot_state = state_handle_->getRobotState();
  const auto coriolis = model_handle_->getCoriolis();
  const auto desired_positions = *desired_positions_buffer_.readFromRT();

  JointArray desired_torques{};
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    desired_torques[i] = k_gains_[i] * (desired_positions[i] - robot_state.q[i]) -
                         d_gains_[i] * robot_state.dq[i] +
                         coriolis_factor_ * coriolis[i];
  }

  const auto saturated_torques = saturateTorqueRate(desired_torques, robot_state.tau_J_d);
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    joint_handles_[i].setCommand(saturated_torques[i]);
  }
}

void JointPositionTorqueController::commandCallback(
    const JointPositionCommandConstPtr& message) {
  JointArray desired_positions{};
  for (std::size_t i = 0; i < kNumberOfJoints; ++i) {
    if (!std::isfinite(message->joint_positions[i])) {
      ROS_ERROR_THROTTLE(
          1.0, "JointPositionTorqueController: rejecting a non-finite position command.");
      return;
    }
    desired_positions[i] = message->joint_positions[i];
  }
  desired_positions_buffer_.writeFromNonRT(desired_positions);
  handleGripperCommand(message->gripper_grasp);
}

void JointPositionTorqueController::handleGripperCommand(bool grasp) {
  if (!gripper_enabled_) {
    return;
  }
  if (received_gripper_command_ && grasp == last_gripper_command_) {
    return;
  }

  if (grasp) 
  {
    if (!gripper_grasp_client_->isServerConnected() &&
        !gripper_grasp_client_->waitForServer(ros::Duration(gripper_server_timeout_))) {
      ROS_WARN_THROTTLE(
          1.0, "JointPositionTorqueController: Franka gripper grasp action is unavailable.");
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
  } 
  else 
  {
    if (!gripper_move_client_->isServerConnected() &&
        !gripper_move_client_->waitForServer(ros::Duration(gripper_server_timeout_))) {
      ROS_WARN_THROTTLE(
          1.0, "JointPositionTorqueController: Franka gripper move action is unavailable.");
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

JointPositionTorqueController::JointArray JointPositionTorqueController::saturateTorqueRate(
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

PLUGINLIB_EXPORT_CLASS(panda_custom_controllers::JointPositionTorqueController,
                       controller_interface::ControllerBase)
