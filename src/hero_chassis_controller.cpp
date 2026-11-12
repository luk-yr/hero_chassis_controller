#include "hero_chassis_controller/hero_chassis_controller.h"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* hw,
    ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  front_left_joint_ = hw->getHandle("left_front_wheel_joint");
  front_right_joint_ = hw->getHandle("right_front_wheel_joint");
  back_left_joint_ = hw->getHandle("left_back_wheel_joint");
  back_right_joint_ = hw->getHandle("right_back_wheel_joint");

  if (!front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left/pid")) ||
      !front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right/pid")) ||
      !back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left/pid")) ||
      !back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right/pid"))) {
    ROS_ERROR("Failed to initialize the wheel PID controllers.");
    return false;
  }

  double initial_velocity;
  controller_nh.param("target_velocity", initial_velocity, 0.0);
  if (!std::isfinite(initial_velocity)) {
    ROS_ERROR("target_velocity must be finite.");
    return false;
  }
  target_velocity_.initRT(initial_velocity);
  command_sub_ = controller_nh.subscribe<std_msgs::Float64>(
      "command", 1, &HeroChassisController::commandCallback, this);

  return true;
}

void HeroChassisController::starting(const ros::Time& time) {
  front_left_pid_.reset();
  front_right_pid_.reset();
  back_left_pid_.reset();
  back_right_pid_.reset();
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  const double target = *target_velocity_.readFromRT();

  front_left_joint_.setCommand(front_left_pid_.computeCommand(
      target - front_left_joint_.getVelocity(), period));
  front_right_joint_.setCommand(front_right_pid_.computeCommand(
      target - front_right_joint_.getVelocity(), period));
  back_left_joint_.setCommand(back_left_pid_.computeCommand(
      target - back_left_joint_.getVelocity(), period));
  back_right_joint_.setCommand(back_right_pid_.computeCommand(
      target - back_right_joint_.getVelocity(), period));
}

void HeroChassisController::stopping(const ros::Time& time) {
  front_left_joint_.setCommand(0.0);
  front_right_joint_.setCommand(0.0);
  back_left_joint_.setCommand(0.0);
  back_right_joint_.setCommand(0.0);
}

void HeroChassisController::commandCallback(
    const std_msgs::Float64ConstPtr& msg) {
  if (std::isfinite(msg->data)) {
    target_velocity_.writeFromNonRT(msg->data);
  } else {
    ROS_WARN("Ignoring a non-finite wheel velocity command.");
  }
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
