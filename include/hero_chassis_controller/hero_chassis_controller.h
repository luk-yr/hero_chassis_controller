#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time,
              const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  void commandCallback(const std_msgs::Float64ConstPtr& msg);

  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;

  control_toolbox::Pid front_left_pid_;
  control_toolbox::Pid front_right_pid_;
  control_toolbox::Pid back_left_pid_;
  control_toolbox::Pid back_right_pid_;

  realtime_tools::RealtimeBuffer<double> target_velocity_;
  ros::Subscriber command_sub_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
