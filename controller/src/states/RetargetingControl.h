#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"
#include <thread>

struct RetargetingControl : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void createGUI(mc_control::fsm::Controller & ctl);
  void createUnityGUI(mc_control::fsm::Controller & ctl);
  void activate(mc_control::fsm::Controller & ctl);
  void deactivate(mc_control::fsm::Controller & ctl);

private:
  std::string state_topic_;
  std::string trigger_topic_;
  std::vector<std::string> states_;
  ros::Publisher retargetting_state_pub_;
  ros::Publisher state_trigger_pub_;
  ROSBoolSubscriber state_trigger_sub_;
  size_t trigger_on_count_ = 0;

  bool trigger_;

  bool active_ = false;
  bool data_online_ = false;
  double maxTime_ = 0.5;
  bool use_vive_trackers_ = false;

  std::shared_ptr<ros::NodeHandle> nh_;
};
