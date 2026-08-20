#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "../ROSSubscriber.h"
#include <ismpc_walking/ControllerConfiguration.h>

struct CoMRetargetting : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void createGUI(mc_control::fsm::Controller & ctl);
  bool calibrate(mc_control::fsm::Controller & ctl);
  void activate(bool state);

private:
  /// @{ CONFIG
  std::string datastorePoseWorld_ = "ChestPose";
  std::string datastoreJoy_ = "LeftHandJoy";
  double minPressTime_ = 0.5; // 2s press to activate
  double scaling_ = 1.0;
  double minCoMHeight_ = 0.6;
  double defaultCoMHeight_ = 0.8;
  double minTorsoPitch_ = mc_rtc::constants::toRad(0);
  double maxTorsoPitch_ = mc_rtc::constants::toRad(15);
  mc_rtc::Configuration config_;

  /// @}
  std::shared_ptr<ros::NodeHandle> nh_;
  std::string state_topic_;
  std::string trigger_topic_;
  ros::Publisher height_state_pub_;
  ros::Publisher state_trigger_pub_;
  size_t trigger_on_count_ = 0;
  ROSBoolSubscriber state_trigger_sub_;
  double maxTime_ = 0.5;

  sva::PTransformd originOffset_ = sva::PTransformd::Identity();

  bool initialized_ = false;
  double previousChestHeight_ = 0.75;
  double initTorsoPitch_ = 0.;
  double chestInitHeight_ = 0.75;
  double comInitHeight_ = 0.8;
  bool active_ = false;
  bool active_trigger_ = false; // trigger the change of state
  bool com_up_ = true;
  bool was_not_active_ = true;
  double comHeight_ = previousChestHeight_;
  double torsoPitch_ = initTorsoPitch_;
  double torsoPitchCoeff_ = 1e-2;

  double moveCoMDuration_ = 3.0;
  size_t moveCoMIter_ = 0;
  bool moveCoMActive_ = false;
  double moveCoMInitHeight_ = 0.8;
  bool moveUp_ = true;
  bool walkingRequested_ = false;
};
