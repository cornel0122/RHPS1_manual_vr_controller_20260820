#pragma once

#include <mc_control/fsm/State.h>
#include <mc_filter/LowPass.h>
#include <mc_tasks/ForceConstrainedTransformTask.h>
#include <mc_tasks/TransformTask.h>
#include "../ROSSubscriber.h"
#include "../motion_prediction/Motion_Prediction.h"
#include <mc_neuron_mocap_plugin/MoCapProperties.h>
#include <thread>

struct OrientationRetargetting : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void createGUI(mc_control::fsm::Controller & ctl);
  void createUnityGUI(mc_control::fsm::Controller & ctl);
  void user_activate(mc_control::fsm::Controller & ctl)
  {
    user_active_ = true;
    activate(ctl);
  }
  void user_deactivate(mc_control::fsm::Controller & ctl)
  {
    user_active_ = false;
    deactivate(ctl);
  }
  void activate(mc_control::fsm::Controller & ctl);
  void deactivate(mc_control::fsm::Controller & ctl);

private:
  /// @{ CONFIG
  std::string datastoreRetargetingReferenceRobot_ = "RetargetingReferenceRobot";
  std::string datastoreRetargetingReferenceOperator_ = "RetargetingReferenceOperator";
  MoCap_Body_part ori_task_body = MoCap_Body_part::LeftForeArm;

  std::string body_topic_;
  std::string reference_topic_;
  std::string body_vel_topic_;
  std::string state_topic_;
  std::string trigger_topic_;
  ros::Publisher retargetting_state_pub_;
  ros::Publisher state_trigger_pub_;
  ROSBoolSubscriber state_trigger_sub_;
  ROSPoseStampedSubscriber reference_pose_sub_;
  ROSAccelStampedSubscriber vel_sub_;

  ROSPoseStampedSubscriber body_pose_sub_;

  bool ignore_emergency_ = false;

  double maxTime_ = 0.5;
  bool use_vive_trackers_ = false;

  double scaling_ = 1.0;

  // For the origin for retargetting we have two choices:
  // - Use the robot head pose
  // - Use the occulus pose (from the datastore)
  std::string originFrame_ = "";
  sva::PTransformd originFrameOffset_ = sva::PTransformd::Identity();
  // Origin from datastore
  std::string datastoreHeadFrame_ = "";

  std::string target_frame_ = "";
  sva::PTransformd X_body_frame = sva::PTransformd::Identity();
  mc_rbdyn::FramePtr frame_;
  sva::PTransformd targetFrameOffset_ = sva::PTransformd::Identity();
  std::vector<std::string> activeJoints_; // If empty use all joints
  Eigen::Vector6d dimWeight_ = Eigen::Vector6d::Ones();
  Eigen::Vector6d dimStiffness_ = Eigen::Vector6d::Ones();
  mc_rtc::Configuration config_;
  /// @}

  /// @{ State
  std::shared_ptr<mc_tasks::TransformTask> body_task_ = nullptr;
  double minStiffness_ = 5;
  double maxStiffness_ = 200;
  size_t body_control_activatedTimestep_ = 0;
  size_t trigger_on_count_ = 0;
  double maxStiffTimeThreshold_ = 10; // Time after which hand task gain reach max [s]
  double linearStiffTimeThreshold_ = 5; // Time after which hand task gain switch from min to gradually reach max [s]
  double weight_ = 1000;
  ///@}

  /// @{ State

  /// @}
  double criticalJumpSafetyDistance_ = 0.1;
  double maxHandDistanceFromReferenceSafety_ = 1.0;
  double maxElbowDistanceFromChestSafety_ = 0.8;
  bool safety_active_ = false;

  bool active_ = false; // Whether the pose from ROS should be tracked
  bool data_online_ = false;
  bool unity_online_ = false;

  bool active_if_possible_ = false;
  bool user_active_ = false;

  bool first_active_ = true; // Set to false once the retargetting is active for the forst time;
  bool trigger_ = false;
  sva::PTransformd X_chest_body_ = sva::PTransformd::Identity();
  sva::PTransformd X_u0_body_ = sva::PTransformd::Identity();
  sva::MotionVecd V_body_ = sva::MotionVecd::Zero(); // Body velocity on body pose in world base
  Eigen::Matrix3d R_0_body_target_ = Eigen::Matrix3d::Identity();
  sva::MotionVecd ref_vel_ = sva::MotionVecd::Zero();

  std::shared_ptr<ros::NodeHandle> nh_;
};
