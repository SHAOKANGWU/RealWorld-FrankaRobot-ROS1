#include <panda_custom_controllers/kinesthetic_teaching_controller.h>

#include <algorithm>
#include <cmath>

#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>


namespace panda_custom_controllers {

// In C++14 this out-of-class definition is required because std::min/std::max
// bind kDeltaTauMax by reference, which odr-uses the static constexpr member.
constexpr double KinestheticTeachingController::kDeltaTauMax;

std::array<double, 7> KinestheticTeachingController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d)
{
    std::array<double, 7> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) 
    {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(
            std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }
    return tau_d_saturated;
}

bool KinestheticTeachingController::init(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& node_handle) 
{
    std::string arm_id;
    std::vector<std::string> joint_names;
    std::vector<double> damping;

    if (!node_handle.getParam("arm_id", arm_id)) 
    {
        ROS_ERROR(
            "KinestheticTeachingController: "
            "Could not read arm_id.");
    return false;
    }

    if (!node_handle.getParam("joint_names", joint_names)) 
    {
        ROS_ERROR(
            "KinestheticTeachingController: "
            "Could not read joint_names.");
        return false;
    }

    if (joint_names.size() != 7) 
    {
        ROS_ERROR_STREAM(
            "KinestheticTeachingController: "
            "Expected 7 joint names, but got "
            << joint_names.size());
        return false;
    }

    if (!node_handle.getParam("damping_", damping)) 
    {
        ROS_ERROR(
            "KinestheticTeachingController: "
            "Could not read damping_ values.");
        return false;
    }

    if (damping.size() != 7) 
    {
        ROS_ERROR_STREAM(
            "KinestheticTeachingController: "
            "Expected 7 damping values, but got "
            << damping.size());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) 
    {
        ROS_ERROR(
            "KinestheticTeachingController: "
            "Could not get EffortJointInterface.");
        return false;
    }
    // get the 7 JointHandles
    joint_handles_.reserve(joint_names.size());
    for (const auto& joint_name : joint_names) 
    {
        try 
        {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
        } catch (
            const hardware_interface::HardwareInterfaceException& e) 
        {
            ROS_ERROR_STREAM(
                "KinestheticTeachingController: "
                "Could not get handle for "
                << joint_name
                << ": "
                << e.what());
            return false;
        }
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR(
            "KinestheticTeachingController: "
            "Could not get FrankaStateInterface.");
        return false;
    }

    try 
    {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    } 
    
    catch (
        const hardware_interface::
            HardwareInterfaceException& e) 
    {
        ROS_ERROR_STREAM(
            "KinestheticTeachingController: "
            "Could not get Franka state handle: "
            << e.what());

        return false;
    }

    // assign damping values to the member variable
    for (size_t i = 0; i < 7; ++i) 
    {
        damping_[i] = damping[i];
    }

    return true;
}



void KinestheticTeachingController::starting(
    const ros::Time& /*time*/) {

  ROS_INFO(
      "KinestheticTeachingController started. "
      "Gravity-compensation / hand-guiding mode active.");
}


void KinestheticTeachingController::update(
    const ros::Time& /*time*/,
    const ros::Duration& /*period*/) {

  // ---------------------------------------------
  // 1. Read current Franka robot state
  // ---------------------------------------------

  const franka::RobotState robot_state =
      state_handle_->getRobotState();


  // ---------------------------------------------
  // 2. Desired user torque = zero
  //
  // Gravity and motor-friction compensation
  // are handled internally by Franka.
  // ---------------------------------------------

  // Desired torque to be applied to the robot joints;
  // Here we use a simple damping controller to
  // provide a smooth hand-guiding experience.
  std::array<double, 7> tau_d_calculated{};
  for (std::size_t i = 0; i < 7; ++i) 
  {
    tau_d_calculated[i] = -damping_[i] * robot_state.dq[i];
  }

  // // Here we use a zero torque command to provide a smooth hand-guiding experience.
  // std::array<double, 7> tau_d_calculated{};
  // tau_d_calculated.fill(0.0);


  // ---------------------------------------------
  // 3. Smoothly approach zero torque
  // ---------------------------------------------

  const std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(
          tau_d_calculated,
          robot_state.tau_J_d);


  // ---------------------------------------------
  // 4. Send torque commands
  // ---------------------------------------------

  for (std::size_t i = 0;
       i < joint_handles_.size();
       ++i) {

    joint_handles_[i].setCommand(
        tau_d_saturated[i]);
  }
}


void KinestheticTeachingController::stopping(
    const ros::Time& /*time*/) {

  ROS_INFO(
      "KinestheticTeachingController stopped.");
}

}  // namespace panda_custom_controllers



PLUGINLIB_EXPORT_CLASS(
    panda_custom_controllers::KinestheticTeachingController,
    controller_interface::ControllerBase)
