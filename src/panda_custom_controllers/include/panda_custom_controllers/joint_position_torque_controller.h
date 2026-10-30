#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <controller_interface/multi_interface_controller.h>
#include <panda_custom_controllers/JointPositionCommand.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>

namespace panda_custom_controllers {

class JointPositionTorqueController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  static constexpr std::size_t kNumberOfJoints = 7;
  using JointArray = std::array<double, kNumberOfJoints>;
  using GripperMoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
  using GripperGraspClient = actionlib::SimpleActionClient<franka_gripper::GraspAction>;

  void commandCallback(const JointPositionCommandConstPtr& message);
  void handleGripperCommand(bool grasp);
  JointArray saturateTorqueRate(const JointArray& desired_torques,
                                const JointArray& previous_desired_torques) const;

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  realtime_tools::RealtimeBuffer<JointArray> desired_positions_buffer_;
  ros::Subscriber command_subscriber_;
  std::unique_ptr<GripperMoveClient> gripper_move_client_;
  std::unique_ptr<GripperGraspClient> gripper_grasp_client_;

  JointArray k_gains_{};
  JointArray d_gains_{};
  double coriolis_factor_{1.0};
  double delta_tau_max_{1.0};
  double gripper_speed_{0.05};
  double gripper_force_{20.0};
  double gripper_open_width_{0.08};
  double gripper_grasp_width_{0.0};
  double gripper_epsilon_inner_{0.005};
  double gripper_epsilon_outer_{0.08};
  double gripper_server_timeout_{0.5};
  bool gripper_enabled_{true};
  bool received_gripper_command_{false};
  bool last_gripper_command_{false};
};

}  // namespace panda_custom_controllers
