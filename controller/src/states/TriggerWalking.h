#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>

struct TriggerWalking : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  /// @{ CONFIG
  std::string datastoreLeftHandJoy_ = "LeftHandJoy";
  std::string datastoreRightHandJoy_ = "RightHandJoy";
  double triggerWalkingThreshold_ = 0.1;
  bool walkingRequested_ = false;
  /// @}
};
