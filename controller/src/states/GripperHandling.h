#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/generic_gripper.h>
#include <mc_rtc/ros.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "../ROSSubscriber.h"

struct GripperHandling : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

  void openCloseThumb(mc_control::Gripper & gripper);
  void openCloseFingers(mc_control::Gripper & gripper);

private:
  /// @{ CONFIG
  std::string datastoreJoy_ = "RightHandJoy";
  std::string gripper_ = "";
  std::string thumbJoint_ = "";
  double thumbPercentOpen_ = 1;
  double thumbPercentClosed_ = 0;
  std::string fingersJoint_ = "";
  double fingersPercentOpen_ = 1;
  double fingersPercentClosed_ = 0.3;
  mc_rtc::Configuration safety_;
  mc_rtc::Configuration config_;
  std::string gripper_opening_topic_;
  ROSFloatSubscriber gripper_openning_sub_;
  /// @}
  std::shared_ptr<ros::NodeHandle> nh_;
  double maxTime_ = 0.5;

  double display_target_opening_ = 1;
  double target_opening_ = 0.5;

  bool closingThumb = true;
  bool closingFingers = true;
};
