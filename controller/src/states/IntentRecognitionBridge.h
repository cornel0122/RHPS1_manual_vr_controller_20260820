#pragma once

#include <mc_control/fsm/State.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct IntentRecognitionBridge : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  struct GoalCandidate
  {
    std::string name;
    std::string object;
    std::string action;
    double belief = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    bool hasPosition = false;
    char hand = 'r';
    double assistanceAlpha = 0.0;
  };

  struct IntentPacket
  {
    std::vector<GoalCandidate> goals;
    std::string frameId;
    double alpha = -1.0;
    double confidence = -1.0;
  };

  enum class Backend
  {
    Guillaume,
    Walid,
    Cornel
  };

  void onExternalIntent(const std_msgs::msg::String::SharedPtr msg, char hand);
  IntentPacket mergedExternalPacket() const;
  bool parseExternalPacket(const std::string & data, IntentPacket & packet, std::string & error) const;
  bool parseObjectGoals(const mc_rtc::Configuration & payload, IntentPacket & packet) const;
  bool parseCornelGoalBeliefs(const mc_rtc::Configuration & payload, IntentPacket & packet) const;
  void finalizePacket(IntentPacket & packet) const;
  bool frameIsSafe(mc_control::fsm::Controller & ctl, const IntentPacket & packet) const;
  bool writePacket(mc_control::fsm::Controller & ctl, const IntentPacket & packet);
  void updateHandPositions(mc_control::fsm::Controller & ctl);
  void publishRobotInputs(mc_control::fsm::Controller & ctl);
  Eigen::Vector3d cameraOpticalToWorld(mc_control::fsm::Controller & ctl,
                                      const Eigen::Vector3d & point) const;
  Eigen::Vector3d worldToCameraOptical(mc_control::fsm::Controller & ctl,
                                      const Eigen::Vector3d & point) const;
  bool isPrimaryAssistanceAction(const std::string & action) const;
  std::string assistanceObjectName(const std::string & object) const;
  bool isAllowedAssistanceObject(const std::string & object) const;
  std::string assistanceGoalName(const GoalCandidate & goal) const;
  void updateGuillaumeStatus(mc_control::fsm::Controller & ctl);

  static std::string normalizedGoalName(const std::string & action, const std::string & object);
  static Backend backendFromString(const std::string & name);

private:
  bool enabled_ = false;
  bool writeAssistanceDatastore_ = false;
  bool allowMissingFrame_ = false;
  bool ownsRclcpp_ = false;
  bool hasPacket_ = false;
  bool packetFrameSafe_ = false;
  bool assistanceDataReady_ = false;
  bool warnedUnsafeFrame_ = false;
  bool warnedParse_ = false;
  bool publishRobotInputs_ = true;
  bool useRobotCameraTransform_ = true;
  bool primaryOnlyAssistance_ = true;
  bool mapPrimaryToGuillaumeGrab_ = true;

  Backend backend_ = Backend::Guillaume;
  std::string backendName_ = "guillaume";
  std::string topic_ = "/goal_beliefs";
  std::string leftTopic_ = "/left_goal_beliefs";
  std::string defaultHand_ = "r";
  std::string leftTaskKey_ = "ANA::LeftHandRetargetingTask";
  std::string rightTaskKey_ = "ANA::RightHandRetargetingTask";
  std::string leftRobotHandFrame_ = "LeftHand";
  std::string rightRobotHandFrame_ = "RightHand";
  std::string handInputSource_ = "robot_frame";
  std::string robotCameraFrame_ = "ZMiniCenter";
  std::string cameraOpticalFrameId_ = "head_camera_optical_frame";
  std::string cameraPoseTopic_ = "/rtgr/robot/camera_pose";
  std::string leftHandCameraTopic_ = "/rtgr/robot/left_hand_point_camera";
  std::string rightHandCameraTopic_ = "/rtgr/robot/right_hand_point_camera";
  std::vector<std::string> acceptedFrames_ = {"robot_world", "world", "BODY"};
  std::vector<std::string> primaryAssistanceActions_ = {"approach_or_grab", "grab"};
  std::vector<std::string> assistanceObjectAllowlist_ = {
      "pitcher", "bottle", "salt", "sauce", "potato1"};
  std::unordered_map<std::string, std::string> assistanceObjectAliases_ = {
      {"potato", "potato1"}};

  double confidenceThreshold_ = 0.2;
  double alphaCap_ = 0.75;
  double positionScale_ = 1.0;
  Eigen::Matrix3d sourceToRobotRotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d sourceToRobotTranslation_ = Eigen::Vector3d::Zero();
  mc_rtc::Configuration config_;

  std::string status_ = "disabled";
  std::string currentGoal_ = "none";
  std::string currentAssistanceGoal_ = "none";
  std::string currentFrame_ = "none";
  double currentAlpha_ = 0.0;
  double currentConfidence_ = 0.0;

  IntentPacket latestPacket_;
  IntentPacket latestRightPacket_;
  IntentPacket latestLeftPacket_;
  bool hasRightPacket_ = false;
  bool hasLeftPacket_ = false;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr intentSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr leftIntentSub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cameraPosePub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr leftHandCameraPub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rightHandCameraPub_;
};
