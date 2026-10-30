#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <controller_interface/multi_interface_controller.h>
#include <panda_custom_controllers/CartesianPoseCommand.h>
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

class CartesianPoseTorqueController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  static constexpr std::size_t kNumberOfJoints = 7;
  using JointArray = std::array<double, kNumberOfJoints>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Jacobian = Eigen::Matrix<double, 6, 7>;
  using GripperMoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
  using GripperGraspClient = actionlib::SimpleActionClient<franka_gripper::GraspAction>;

  // Plain arrays keep commands copied through RealtimeBuffer independent of Eigen alignment.
  struct PoseCommand {
    std::array<double, 3> position{};
    // Quaternion coefficient order: x, y, z, w.
    std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
  };

  void commandCallback(const CartesianPoseCommandConstPtr& message);
  void handleGripperCommand(bool grasp);
  JointArray saturateTorqueRate(const JointArray& desired_torques,
                                const JointArray& previous_desired_torques) const;

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  realtime_tools::RealtimeBuffer<PoseCommand> desired_pose_buffer_;
  ros::Subscriber command_subscriber_;
  std::unique_ptr<GripperMoveClient> gripper_move_client_;
  std::unique_ptr<GripperGraspClient> gripper_grasp_client_;

  Matrix6d cartesian_stiffness_{Matrix6d::Zero()};
  Matrix6d cartesian_damping_{Matrix6d::Zero()};
  std::string base_frame_;
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
