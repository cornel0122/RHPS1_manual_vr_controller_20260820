#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/ForceConstrainedTransformTask.h>
#include <mc_tasks/TransformTask.h>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vr_interface_msgs/msg/controller.hpp>
#include <vr_interface_msgs/msg/tracker.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

struct ViveHandBridge : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  using Controller = vr_interface_msgs::msg::Controller;
  using Tracker = vr_interface_msgs::msg::Tracker;

  template<typename MsgT>
  struct TimedSample
  {
    double time = 0.0;
    MsgT msg;
  };

  struct HandState
  {
    struct VelocityDiagnostics
    {
      Eigen::Vector3d targetPositionWorld = Eigen::Vector3d::Zero();
      Eigen::Vector3d linearWorld = Eigen::Vector3d::Zero();
      Eigen::Vector3d angularWorld = Eigen::Vector3d::Zero();
      Eigen::Vector3d linearCommandWorldRequested = Eigen::Vector3d::Zero();
      Eigen::Vector3d angularCommandWorldRequested = Eigen::Vector3d::Zero();
      Eigen::Vector3d linearCommandLocal = Eigen::Vector3d::Zero();
      Eigen::Vector3d angularCommandLocal = Eigen::Vector3d::Zero();
      Eigen::Vector3d linearCommandWorldTaskFrame = Eigen::Vector3d::Zero();
      Eigen::Vector3d angularCommandWorldTaskFrame = Eigen::Vector3d::Zero();
      double linearDirectionCosine = 0.0;
      double linearMagnitudeRatio = 0.0;
      double angularDirectionCosine = 0.0;
      double angularMagnitudeRatio = 0.0;
      bool sampleValid = false;
      bool linearComparisonValid = false;
      bool angularComparisonValid = false;
    };

    std::string serial;
    std::string topic;
    rclcpp::Subscription<Controller>::SharedPtr sub;
    Controller latest;
    std::deque<TimedSample<Controller>> history;
    double latestSampleTime = 0.0;
    bool hasData = false;
    bool zeroed = false;
    bool previousResetButton = false;
    bool previousGripperButton = false;
    bool previousGraspNotificationButton = false;
    bool gripperClosed = false;
    Eigen::Vector3d zeroVive = Eigen::Vector3d::Zero();
    Eigen::Matrix3d zeroRotation = Eigen::Matrix3d::Identity();
    sva::PTransformd zeroRobot = sva::PTransformd::Identity();
    sva::PTransformd previousTarget = sva::PTransformd::Identity();
    double previousTargetTime = 0.0;
    bool previousTargetValid = false;
    sva::MotionVecd heldTargetVelocity = sva::MotionVecd::Zero();
    VelocityDiagnostics velocityDiagnostics;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr legacyPosePub;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr legacyVelocityPub;
    std::shared_ptr<mc_tasks::ForceConstrainedTransformTask> task;
  };

  struct TrackerState
  {
    std::string serial;
    std::string topic;
    rclcpp::Subscription<Tracker>::SharedPtr sub;
    Tracker latest;
    std::deque<TimedSample<Tracker>> history;
    double latestSampleTime = 0.0;
    bool hasData = false;
    bool zeroed = false;
    Eigen::Vector3d zeroPosition = Eigen::Vector3d::Zero();
    Eigen::Matrix3d zeroRotation = Eigen::Matrix3d::Identity();
    sva::PTransformd zeroRobot = sva::PTransformd::Identity();
    sva::PTransformd previousTarget = sva::PTransformd::Identity();
    double previousTargetTime = 0.0;
    bool previousTargetValid = false;
    sva::MotionVecd heldTargetVelocity = sva::MotionVecd::Zero();
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr legacyPosePub;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr legacyVelocityPub;
    std::shared_ptr<mc_tasks::TransformTask> task;
  };

  void onController(HandState & hand, const Controller::SharedPtr msg);
  void onTracker(TrackerState & tracker, const Tracker::SharedPtr msg);
  void onAnyController(const std::string & topic, const Controller::SharedPtr msg);
  void onAnyTracker(const std::string & topic, const Tracker::SharedPtr msg);
  void updateDelayedInputs();
  double now() const;
  void pushSample(HandState & state, const Controller & msg);
  void pushSample(TrackerState & state, const Tracker & msg);
  template<typename StateT>
  void selectDelayedSample(StateT & state);
  Eigen::Vector3d vivePosition(const Controller & msg) const;
  Eigen::Matrix3d viveRotation(const Controller & msg) const;
  Eigen::Vector3d controllerHandOffset(const HandState & hand) const;
  Eigen::Vector3d trackerPosition(const Tracker & msg) const;
  Eigen::Matrix3d trackerRotation(const Tracker & msg) const;
  Eigen::Vector3d relativeHandPosition(const HandState & hand) const;
  Eigen::Matrix3d relativeHandRotation(const HandState & hand) const;
  Eigen::Matrix3d relativeTrackerRotation(const TrackerState & tracker) const;
  Eigen::Vector3d mapDeltaToRobot(const Eigen::Vector3d & delta, bool leftHand, bool applyMaxStep = true) const;
  Eigen::Vector3d mapWaistRelativePositionToRobot(const Eigen::Vector3d & position, bool leftHand) const;
  Eigen::Matrix3d rpyDegreesToMatrix(const Eigen::Vector3d & rpyDegrees) const;
  Eigen::Vector3d rotationVectorFromMatrix(const Eigen::Matrix3d & rotation) const;
  Eigen::Matrix3d matrixFromRotationVector(const Eigen::Vector3d & rotationVector) const;
  Eigen::Matrix3d mapHandRotationDeltaToRobot(const Eigen::Matrix3d & delta,
                                              const Eigen::Matrix3d & sideCorrection,
                                              bool leftHand) const;
  Eigen::Matrix3d mapArmRotationDeltaToRobot(const Eigen::Matrix3d & delta) const;
  sva::PTransformd blendFromTrackingHold(const sva::PTransformd & held, const sva::PTransformd & target) const;
  void setZero(mc_control::fsm::Controller & ctl);
  void updateHand(mc_control::fsm::Controller & ctl, HandState & hand);
  void updateArm(mc_control::fsm::Controller & ctl, TrackerState & tracker);
  sva::MotionVecd computeTargetVelocity(sva::PTransformd & previousTarget,
                                         double & previousTargetTime,
                                         bool & previousTargetValid,
                                         sva::MotionVecd & heldTargetVelocity,
                                         const sva::PTransformd & target,
                                         double sampleTime,
                                         HandState::VelocityDiagnostics * diagnostics = nullptr);
  void updateVelocityDiagnostics(HandState & hand,
                                 const sva::PTransformd & target,
                                 const sva::MotionVecd & worldVelocity,
                                 const sva::MotionVecd & commandWorldVelocity);
  void setupVelocityValidation(mc_control::fsm::Controller & ctl);
  void addVelocityValidationLogs(mc_control::fsm::Controller & ctl,
                                 HandState & hand,
                                 const std::string & side);
  sva::MotionVecd feedforwardVelocity(const sva::MotionVecd & velocity) const;
  void publishLegacyPoseVelocity(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & posePub,
                                 const rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr & velocityPub,
                                 const sva::PTransformd & target,
                                 const sva::MotionVecd & velocity);
  void updateGrippers(mc_control::fsm::Controller & ctl);
  void publishAdhesionCommand();
  void publishGraspEvent(const std::string & hand, const std::string & event, double triggerValue);
  bool resetPressed(HandState & hand) const;
  bool gripperPressed(HandState & hand) const;

private:
  std::shared_ptr<rclcpp::Node> node_;
  bool ownsRclcpp_ = false;
  bool zeroRequested_ = false;
  bool autoZero_ = true;
  bool autoZeroDone_ = false;
  bool requireInitialTwoHandReset_ = true;
  bool initialZeroRequiresArmTrackers_ = true;
  bool trackingStarted_ = false;
  bool initialLeftResetSeen_ = false;
  bool initialRightResetSeen_ = false;
  bool initialWaitingForDataLogged_ = false;
  double trackingHandoverStartTime_ = -1.0;
  double trackingHandoverDuration_ = 2.0;

  HandState left_;
  HandState right_;
  TrackerState leftArm_;
  TrackerState rightArm_;
  TrackerState waist_;
  std::vector<rclcpp::Subscription<Controller>::SharedPtr> controllerSubs_;
  std::vector<rclcpp::Subscription<Tracker>::SharedPtr> trackerSubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr adhesionPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graspEventPub_;

  std::string leftTaskName_ = "ANA::LeftHandRetargetingTask";
  std::string rightTaskName_ = "ANA::RightHandRetargetingTask";
  std::string leftArmTaskName_ = "ANA::LeftArmRetargetingTask";
  std::string rightArmTaskName_ = "ANA::RightArmRetargetingTask";
  std::string leftGripper_ = "l_gripper";
  std::string rightGripper_ = "r_gripper";
  std::string leftGripperJoint_ = "L_HAND_THUMB_J0";
  std::string rightGripperJoint_ = "R_HAND_THUMB_J0";
  std::string robotWaistFrame_ = "BODY";
  std::string leftRobotHandFrame_ = "LeftHand";
  std::string rightRobotHandFrame_ = "RightHand";
  mc_rtc::Configuration config_;

  double scale_ = 1.0;
  Eigen::Vector3d axisScale_ = Eigen::Vector3d::Ones();
  Eigen::Vector3d waistReferenceOffsetBody_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d handPositionOffsetBody_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftControllerHandOffset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightControllerHandOffset_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d waistToRobotPosition_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d neutralBodyFromWaist_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d leftHandNeutralCorrection_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rightHandNeutralCorrection_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d leftArmNeutralCorrection_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rightArmNeutralCorrection_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d directHandOrientationAxisFix_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d directHandRotationSignFix_ = Eigen::Vector3d::Ones();
  int directHandRotationAxisMode_ = 0;
  Eigen::Matrix3d leftHandOrientationCorrection_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rightHandOrientationCorrection_ = Eigen::Matrix3d::Identity();
  double maxStep_ = 0.35;
  double directWaistMaxDistance_ = 1.2;
  double inputDelay_ = 0.0;
  double openValue_ = 0.7;
  double closedValue_ = 0.05;
  double adhesionOpenValue_ = 0.0;
  double adhesionClosedValue_ = 0.01;
  double graspNotificationTriggerThreshold_ = 0.9;
  double velocityFeedforwardGain_ = 1.0;
  double linearVelocityGain_ = 1.0;
  double angularVelocityGain_ = 1.0;
  double velocityValidationArrowScale_ = 0.5;
  double velocityValidationMinSpeed_ = 0.01;
  double velocityFilterAlpha_ = 0.7;
  double velocitySampleTimeout_ = 0.1;
  bool leftPositionHorizontalFix_ = true;
  int leftPositionHorizontalMode_ = 2;
  bool debugHandMapping_ = false;
  double debugHandMappingInterval_ = 1.0;
  double lastHandMappingDebugTime_ = 0.0;
  bool controlGrippers_ = true;
  bool controlMujocoAdhesion_ = true;
  bool controlHandOrientation_ = true;
  bool useVelocityFeedforward_ = true;
  bool useLinearVelocityFeedforward_ = true;
  bool useAngularVelocityFeedforward_ = false;
  bool velocityValidation_ = false;
  bool publishLegacyUnityTopics_ = true;
  bool neutralBodyCalibration_ = false;
  bool wristPivotCompensation_ = false;
  bool directWaistKeepHandSide_ = true;
  bool invertHandOrientationDelta_ = true;
  bool invertArmOrientationDelta_ = false;
  Eigen::Vector3d armRotationSignFix_ = Eigen::Vector3d::Ones();
  Eigen::Vector3d armAngularVelocitySignFix_ = Eigen::Vector3d::Ones();
  int leftHandRotationAxisMode_ = 0;
  int rightHandRotationAxisMode_ = 1;
  bool useWaistReference_ = true;
  bool directWaistControl_ = true;
  std::string adhesionTopic_ = "/ana/mujoco/adhesion";
  std::string graspEventTopic_ = "/rtgr/operator/grasp_events";
  uint64_t graspEventSequence_ = 0;
  std::vector<std::string> velocityValidationLogEntries_;
};
