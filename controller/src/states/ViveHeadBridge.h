#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <rclcpp/rclcpp.hpp>
#include <vr_interface_msgs/msg/tracker.hpp>

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

  void onHmd(const Tracker::SharedPtr msg);
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
  std::shared_ptr<mc_tasks::TransformTask> task_;

  Tracker latest_;
  bool hasData_ = false;
  bool zeroed_ = false;
  Eigen::Matrix3d zeroHmd_ = Eigen::Matrix3d::Identity();
  sva::PTransformd zeroRobot_ = sva::PTransformd::Identity();

  bool enable_ = true;
  bool requireExternalZero_ = false;
  int headRotationAxisMode_ = 0;
  double headPitchGain_ = 1.0;
};
