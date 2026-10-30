#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>

namespace panda_custom_controllers {

struct CartesianVelocityCommand {
  std::array<double, 6> velocity{};
  ros::Time stamp;
};

class CartesianVelocityTeleopController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaVelocityCartesianInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time,
              const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  void commandCallback(
      const geometry_msgs::TwistStampedConstPtr& msg);

  franka_hw::FrankaVelocityCartesianInterface*
      velocity_cartesian_interface_{nullptr};

  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle>
      velocity_cartesian_handle_;

  ros::Subscriber command_subscriber_;

  realtime_tools::RealtimeBuffer<CartesianVelocityCommand>
      command_buffer_;

  std::array<double, 6> commanded_velocity_{};

  std::string command_frame_{"panda_link0"};

  double command_timeout_{0.20};

  double max_linear_velocity_{0.03};
  double max_angular_velocity_{0.20};

  double max_linear_acceleration_{0.10};
  double max_angular_acceleration_{0.50};
};

}  // namespace panda_custom_controllers