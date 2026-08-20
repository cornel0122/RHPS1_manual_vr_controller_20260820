#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "../ROSSubscriber.h"
#include <condition_variable>

struct ROSInputState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void simulateInput(mc_control::fsm::Controller & ctl);
  void getInput(mc_control::fsm::Controller & ctl);

private:
  std::string face_topic_ = "/avatar/face";
  std::string hands_joy_topic_ = "/avatar/joy";
  std::string emergency_button_topic_ = "/emergency_button";
  std::string trackpad_topic_ = "/avatar/trackpad";

  double maxTime_ = 0.5;

  double ros_frequency = 60;

  ROSBoolSubscriber emergency_button_sub_;

  bool emergency_button_connected_ = true;
  double emergency_button_max_time_ = 10;
  bool emergency_override_ = false;

  ROSMultiArraySubscriber face_sub_;
  double face_sub_max_time_ = 1;

  ROSOcculusLeftHandJoySubscriber left_hand_joy_sub_;
  OcculusHandJoystick left_hand_joy_;
  OcculusStatefulJoystick left_hand_joy_state_;

  ROSOcculusRightHandJoySubscriber right_hand_joy_sub_;
  OcculusHandJoystick right_hand_joy_;
  OcculusStatefulJoystick right_hand_joy_state_;

  ROSPS4JoySubscriber ps4_joy_sub_;

  ROSMultiArraySubscriber trackpad_sub_;

  /** Simulation
   * @{ */
  bool simulation_ = false;
  std::string leftHandFrame_ = "LeftHand";
  std::string rightHandFrame_ = "RightHand";
  std::string headFrame_ = "ZMiniCenter";
  /* @} */

private:
  /**
   * Handing the ROS thread
   * @{
   */
  std::shared_ptr<ros::NodeHandle> nh_;
  std::thread spinThread_;
  bool running_ = true;
  void rosSpinner();
  /// @}
};
