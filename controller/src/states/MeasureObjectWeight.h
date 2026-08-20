#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/io_utils.h>
#include "../ANAAvatarController.h"
#include "../ROSSubscriber.h"

struct MeasureObjectWeight : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double offset_;
  double contact_threshold_ = 0.05; // limit to which we consider internal forces

private:
  ros::Publisher weight_pub_;
  ros::Publisher force_pub_;
  std::shared_ptr<ros::NodeHandle> nh_;
  /// @{ CONFIG
  std::string pub_topic_;
  std::string finger_force_topic_;
  std::string frame_name_;
  std::string contact_frame_; // name of a frame to which it can encounters internal forces;
  std::string forceSensorName_ = "RightHandForceSensor";
  ///@}
  sva::ForceVecd measured_wrench_;
  float weight_;
};
