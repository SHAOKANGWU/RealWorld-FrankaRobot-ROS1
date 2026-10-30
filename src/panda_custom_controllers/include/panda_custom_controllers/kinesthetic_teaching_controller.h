#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <franka_hw/franka_state_interface.h>

#include <ros/ros.h>


namespace panda_custom_controllers {


class KinestheticTeachingController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {

 public:

  bool init(
      hardware_interface::RobotHW* robot_hw,
      ros::NodeHandle& node_handle
  ) override;


  void starting(
      const ros::Time& time
  ) override;


  void update(
      const ros::Time& time,
      const ros::Duration& period
  ) override;


  void stopping(
      const ros::Time& time
  ) override;


 private:
  std::array<double, 7> damping_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Handles used to send torque commands
  // to panda_joint1 ... panda_joint7.
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Gives us the complete Franka RobotState.
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  // Maximum change in commanded torque
  // during one 1-kHz control cycle.
  static constexpr double kDeltaTauMax{1.0};

  std::array<double, 7> saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d);

};


}  // namespace panda_custom_controllers