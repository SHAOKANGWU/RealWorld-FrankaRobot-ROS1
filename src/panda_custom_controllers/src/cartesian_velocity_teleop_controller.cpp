#include <panda_custom_controllers/cartesian_velocity_teleop_controller.h>

#include <algorithm>
#include <cmath>

#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

namespace panda_custom_controllers {

bool CartesianVelocityTeleopController::init(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& node_handle) {

  // --------------------------------------------------
  // Parameters
  // --------------------------------------------------

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR(
        "CartesianVelocityTeleopController: "
        "Could not get arm_id.");
    return false;
  }

  std::string command_topic;

  node_handle.param<std::string>(
      "command_topic",
      command_topic,
      "/teleop/cartesian_velocity_cmd");

  node_handle.param<std::string>(
      "command_frame",
      command_frame_,
      "panda_link0");

  node_handle.param(
      "command_timeout",
      command_timeout_,
      0.20);

  node_handle.param(
      "max_linear_velocity",
      max_linear_velocity_,
      0.03);

  node_handle.param(
      "max_angular_velocity",
      max_angular_velocity_,
      0.20);

  node_handle.param(
      "max_linear_acceleration",
      max_linear_acceleration_,
      0.10);

  node_handle.param(
      "max_angular_acceleration",
      max_angular_acceleration_,
      0.50);

  if (arm_id.empty() || command_topic.empty() || command_frame_.empty()) {
    ROS_ERROR(
        "CartesianVelocityTeleopController: arm_id, command_topic, and "
        "command_frame must not be empty.");
    return false;
  }

  if (command_timeout_ <= 0.0 || max_linear_velocity_ < 0.0 ||
      max_angular_velocity_ < 0.0 || max_linear_acceleration_ <= 0.0 ||
      max_angular_acceleration_ <= 0.0) {
    ROS_ERROR(
        "CartesianVelocityTeleopController: invalid timeout or motion limit; "
        "timeout/accelerations must be positive and velocities non-negative.");
    return false;
  }


  // --------------------------------------------------
  // Franka Cartesian velocity interface
  // --------------------------------------------------

  velocity_cartesian_interface_ =
      robot_hw->get<
          franka_hw::FrankaVelocityCartesianInterface>();

  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityTeleopController: "
        "Could not get Cartesian velocity interface.");
    return false;
  }

  try {
    velocity_cartesian_handle_ =
        std::make_unique<
            franka_hw::FrankaCartesianVelocityHandle>(
            velocity_cartesian_interface_->getHandle(
                arm_id + "_robot"));
  } catch (
      const hardware_interface::HardwareInterfaceException& e) {

    ROS_ERROR_STREAM(
        "CartesianVelocityTeleopController: "
        "Could not get Cartesian velocity handle: "
        << e.what());

    return false;
  }


  // --------------------------------------------------
  // ROS subscriber
  // --------------------------------------------------

  command_subscriber_ =
      node_handle.subscribe(
          command_topic,
          1,
          &CartesianVelocityTeleopController::
              commandCallback,
          this);

  ROS_INFO(
      "CartesianVelocityTeleopController "
      "initialized successfully.");

  return true;
}


void CartesianVelocityTeleopController::commandCallback(
    const geometry_msgs::TwistStampedConstPtr& msg) {

  // --------------------------------------------------
  // Verify coordinate frame
  // --------------------------------------------------

  if (!msg->header.frame_id.empty() &&
      msg->header.frame_id != command_frame_) {

    ROS_WARN_THROTTLE(
        1.0,
        "Cartesian velocity command ignored: "
        "unexpected frame.");

    return;
  }


  CartesianVelocityCommand command;

  command.velocity = {
      msg->twist.linear.x,
      msg->twist.linear.y,
      msg->twist.linear.z,
      msg->twist.angular.x,
      msg->twist.angular.y,
      msg->twist.angular.z
  };


  // --------------------------------------------------
  // Reject NaN / Inf
  // --------------------------------------------------

  for (double value : command.velocity) {
    if (!std::isfinite(value)) {

      ROS_WARN_THROTTLE(
          1.0,
          "Cartesian velocity command ignored: "
          "non-finite value.");

      return;
    }
  }


  // --------------------------------------------------
  // Limit translational velocity norm
  // --------------------------------------------------

  const double linear_norm =
      std::sqrt(
          command.velocity[0] * command.velocity[0] +
          command.velocity[1] * command.velocity[1] +
          command.velocity[2] * command.velocity[2]);

  if (linear_norm > max_linear_velocity_) {

    const double scale =
        max_linear_velocity_ / linear_norm;

    command.velocity[0] *= scale;
    command.velocity[1] *= scale;
    command.velocity[2] *= scale;
  }


  // --------------------------------------------------
  // Limit rotational velocity norm
  // --------------------------------------------------

  const double angular_norm =
      std::sqrt(
          command.velocity[3] * command.velocity[3] +
          command.velocity[4] * command.velocity[4] +
          command.velocity[5] * command.velocity[5]);

  if (angular_norm > max_angular_velocity_) {

    const double scale =
        max_angular_velocity_ / angular_norm;

    command.velocity[3] *= scale;
    command.velocity[4] *= scale;
    command.velocity[5] *= scale;
  }


  // Use reception time for watchdog.
  command.stamp = ros::Time::now();


  // --------------------------------------------------
  // Non-RT → RT
  // --------------------------------------------------

  command_buffer_.writeFromNonRT(command);
}

void CartesianVelocityTeleopController::starting(
    const ros::Time& time) {

  commanded_velocity_.fill(0.0);

  CartesianVelocityCommand initial_command;

  initial_command.velocity.fill(0.0);
  initial_command.stamp = time;

  command_buffer_.initRT(
      initial_command);

  ROS_INFO(
      "CartesianVelocityTeleopController started.");
}



void CartesianVelocityTeleopController::update(
    const ros::Time& time,
    const ros::Duration& period) {

  const CartesianVelocityCommand* command =
      command_buffer_.readFromRT();

  if (command == nullptr) {
    return;
  }

  std::array<double, 6> target_velocity{};

  target_velocity.fill(0.0);


  // --------------------------------------------------
  // Watchdog
  // --------------------------------------------------

  const double command_age =
      (time - command->stamp).toSec();

  if (command_age >= 0.0 &&
      command_age <= command_timeout_) {

    target_velocity =
        command->velocity;
  }


  const double dt =
      period.toSec();

  if (dt <= 0.0) {
    return;
  }


  // --------------------------------------------------
  // Smooth translational acceleration
  // --------------------------------------------------

  double linear_delta[3];

  double linear_delta_norm = 0.0;

  for (std::size_t i = 0; i < 3; ++i) {

    linear_delta[i] =
        target_velocity[i]
        - commanded_velocity_[i];

    linear_delta_norm +=
        linear_delta[i] * linear_delta[i];
  }

  linear_delta_norm =
      std::sqrt(linear_delta_norm);

  const double max_linear_step =
      max_linear_acceleration_ * dt;

  double linear_scale = 1.0;

  if (linear_delta_norm > max_linear_step &&
      linear_delta_norm > 0.0) {

    linear_scale =
        max_linear_step /
        linear_delta_norm;
  }

  for (std::size_t i = 0; i < 3; ++i) {

    commanded_velocity_[i] +=
        linear_scale * linear_delta[i];
  }


  // --------------------------------------------------
  // Smooth rotational acceleration
  // --------------------------------------------------

  double angular_delta[3];

  double angular_delta_norm = 0.0;

  for (std::size_t i = 0; i < 3; ++i) {

    angular_delta[i] =
        target_velocity[i + 3]
        - commanded_velocity_[i + 3];

    angular_delta_norm +=
        angular_delta[i] *
        angular_delta[i];
  }

  angular_delta_norm =
      std::sqrt(angular_delta_norm);

  const double max_angular_step =
      max_angular_acceleration_ * dt;

  double angular_scale = 1.0;

  if (angular_delta_norm > max_angular_step &&
      angular_delta_norm > 0.0) {

    angular_scale =
        max_angular_step /
        angular_delta_norm;
  }

  for (std::size_t i = 0; i < 3; ++i) {

    commanded_velocity_[i + 3] +=
        angular_scale *
        angular_delta[i];
  }


  // --------------------------------------------------
  // Send Cartesian velocity command
  // --------------------------------------------------

  velocity_cartesian_handle_->setCommand(
      commanded_velocity_);
}


void CartesianVelocityTeleopController::stopping(
    const ros::Time& /*time*/) {

  // Do not send a sudden zero velocity here.
  // Let Franka's stopping behavior handle controller stop.

  ROS_INFO(
      "CartesianVelocityTeleopController stopping.");

  commanded_velocity_.fill(0.0);
}



}  // namespace panda_custom_controllers


PLUGINLIB_EXPORT_CLASS(
    panda_custom_controllers::CartesianVelocityTeleopController,
    controller_interface::ControllerBase)
