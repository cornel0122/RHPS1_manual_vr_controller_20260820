#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <rclcpp/rclcpp.hpp>
#include <vr_interface_msgs/msg/tracker.hpp>

#include <deque>
#include <memory>
#include <string>

struct ViveHeadBridge : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  using Tracker = vr_interface_msgs::msg::Tracker;

  struct TimedTracker
  {
    double time = 0.0;
    Tracker msg;
  };

  void onHmd(const Tracker::SharedPtr msg);
  void syncExperimentInput(mc_control::fsm::Controller & ctl);
  void selectDelayedHmd();
  double now() const;
  Eigen::Matrix3d viveRotation(const Tracker & msg) const;
  Eigen::Matrix3d mapRotationDeltaToRobot(const Eigen::Matrix3d & delta) const;
  Eigen::Matrix3d applyHeadRotationAxisMode(const Eigen::Matrix3d & mappedDelta,
                                            const Eigen::Matrix3d & zeroRobotRotation) const;
  void setZero();

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<Tracker>::SharedPtr hmdSub_;
  bool ownsRclcpp_ = false;

  std::string topic_ = "/hmd";
  std::string taskName_ = "ANA::HeadRetargetingTask";
  std::string zeroRequestKey_ = "ANA::ViveHeadBridge::zero_request";
  std::string experimentDelayKey_ = "ANA::Experiment::input_delay";
  std::string experimentSessionKey_ = "ANA::Experiment::session_id";
  std::string experimentConditionReadyKey_ = "ANA::Experiment::condition_received";
  std::string experimentSessionId_ = "none";
  std::shared_ptr<mc_tasks::TransformTask> task_;

  Tracker latest_;
  std::deque<TimedTracker> history_;
  bool hasData_ = false;
  bool zeroed_ = false;
  bool useExperimentDelay_ = false;
  bool experimentConditionReady_ = false;
  double inputDelay_ = 0.0;
  Eigen::Matrix3d zeroHmd_ = Eigen::Matrix3d::Identity();
  sva::PTransformd zeroRobot_ = sva::PTransformd::Identity();

  bool enable_ = true;
  bool requireExternalZero_ = false;
  int headRotationAxisMode_ = 0;
  double headPitchGain_ = 1.0;
};
