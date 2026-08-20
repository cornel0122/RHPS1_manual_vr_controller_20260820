#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "../ROSSubscriber.h"

struct RetargetingReference : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::string torsoFrame_ = "";
  std::string datastoreNameRobot_ = "RobotRetargetingReference";
  std::string datastoreNameOperator_ = "OperatorRetargetingReference";
  std::string reference_frame_operator_topic_;
  sva::PTransformd X_0_referenceRobot_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_referenceOperator_ = sva::PTransformd::Identity();
  sva::PTransformd originFrameOffset_ = sva::PTransformd::Identity();
  mc_rtc::Configuration config_;
  std::string camera_frame_;
  std::string camera_pose_topic_ = "avatar/zed_relative_pose";
  ROSPoseStampedSubscriber operator_pose_sub_;
  ros::Publisher camera_pose_pub_;
  bool unity_online_ = false;
  bool data_online_ = false;
};
