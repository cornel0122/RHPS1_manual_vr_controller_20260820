#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/ros.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include "../ROSSubscriber.h"

struct ROSPublisherState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string unity_pose_topic_ = "/mc_rtc_init_pose_unity";
  ros::Publisher unity_pose_pub_;
  std::string unity_string_topic_ = "/avatar/hud_string";
  ros::Publisher unity_string_pub_;
  std::string msg_ = "";

  std::string unity_head_tracking_topic_ = "/avatar/hud_head";
  ros::Publisher unity_head_pub_;
  std::string unity_left_tracking_topic_ = "/avatar/hud_left";
  ros::Publisher unity_left_pub_;
  std::string unity_right_tracking_topic_ = "/avatar/hud_right";
  ros::Publisher unity_right_pub_;
  std::string unity_com_tracking_topic_ = "/avatar/hud_com";
  ros::Publisher unity_com_pub_;
  std::string unity_walking_topic_ = "/avatar/hud_walking";
  ros::Publisher unity_walking_pub_;
  std::string unity_weight_topic_ = "/avatar/object_weights";
  ros::Publisher unity_weight_pub_;
  std::string unity_left_hand_force_topic_ = "/avatar/left_hand_force";
  ros::Publisher unity_left_hand_force_pub_;
  std::string unity_right_hand_force_topic_ = "/avatar/right_hand_force";
  ros::Publisher unity_right_hand_force_pub_;

  ros::Publisher emergency_pub;
  std::string emergency_topic_ = "avatar/state/emergency";

  std::string cameraFrame_ = "ZMiniCenter";
  bool useDatastore_ = false;
  std::string datastoreFrame_ = "HeadPoseWorld";

  double rate_ = 30;
  unsigned skip_;
  unsigned seq_ = 0;

private:
  /**
   * Handing the ROS thread
   * @{
   */
  std::shared_ptr<ros::NodeHandle> nh_;
  /// @}
};
