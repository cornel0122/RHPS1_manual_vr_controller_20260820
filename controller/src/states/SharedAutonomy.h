#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/LookAtTask.h>

struct SharedAutonomy : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool sharedAutonomyDataReady(mc_control::fsm::Controller & ctl);
  void fallbackToManualRetargetting(mc_control::fsm::Controller & ctl);
  void addExperimentLogs(mc_control::fsm::Controller & ctl);
  void updateAssistanceMetrics(mc_control::fsm::Controller & ctl,
                               bool active,
                               bool translationActive,
                               bool orientationActive,
                               double effectiveAlpha);

  /// @{ CONFIG
  bool enabled_ = true;
  bool useExperimentAssistanceGate_ = false;
  bool experimentGateOpen_ = false;
  // Single-target blending (default): the assistance is a rate-limited position offset
  // alpha * (goal - operator target) added to the operator's own hand target, so the hand
  // keeps one task and the arm never jumps when alpha or the goal changes.
  // false = legacy dual weighted QP tasks.
  bool singleTargetBlending_ = true;
  double assistOffsetMaxSpeed_ = 0.3; // [m/s]
  Eigen::Vector3d assistOffset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanTargetPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d blendedTargetPosition_ = Eigen::Vector3d::Zero();
  // The operator target is read from the hand task. If no other state rewrote the task
  // since this state last wrote the blended target, reuse the remembered operator target
  // instead of treating the blended one as operator input (which would accumulate).
  sva::PTransformd humanTarget_ = sva::PTransformd::Identity();
  sva::PTransformd lastHumanTarget_ = sva::PTransformd::Identity();
  sva::PTransformd lastWrittenTarget_ = sva::PTransformd::Identity();
  bool lastWrittenValid_ = false;
  // Human-readable assistance state for the GUI/log; every change is logged so that a
  // condition with assistance never silently runs as manual teleoperation.
  std::string assistanceStatus_ = "starting";
  void setAssistanceStatus(const std::string & status, bool problem);
  void readHumanTarget(mc_control::fsm::Controller & ctl);
  void applyAssistOffset(mc_control::fsm::Controller & ctl, const Eigen::Vector3d & desiredOffset);
  // Shadow mode (single-target blending only): the assistance is computed with the same
  // dynamics and logged as computed-offset, but the robot's applied offset stays at zero.
  //  - apply_assistance=false: shadow even in conditions with assistance (first robot trials).
  //  - shadow_when_assistance_off=true: conditions without assistance still log what the
  //    assistance would have done, for offline comparison.
  bool applyAssistance_ = true;
  bool shadowWhenAssistanceOff_ = true;
  bool shadow_ = false;
  Eigen::Vector3d computedOffset_ = Eigen::Vector3d::Zero();
  void commandAssist(mc_control::fsm::Controller & ctl, const Eigen::Vector3d & desiredOffset);
  std::string experimentAssistanceKey_ = "ANA::Experiment::assistance_enabled";
  bool warnedMissingData_ = false;
  double alpha;
  int currentGoal;
  Eigen::Vector3d targetPosition;
  Eigen::Vector3d targetOrientationDown;
  Eigen::Quaternion<double> targetOrientation;
  std::vector<double> targetPositions;
  std::vector<double> targetOrientations;
  double limitDistanceOrientation = -10.00;
  double limitDistanceTranslation = -10.00;
  std::unordered_map<int, std::vector<double>> mapOfGoalPositions;
  std::unordered_map<int, std::vector<double>> mapOfGoalOrientations;
  std::string handToChoose;
  std::vector<std::string> activeJoints_; // If empty use all joints
  Eigen::Vector6d dimWeight_ = Eigen::Vector6d::Ones();
  Eigen::Vector6d dimStiffness_ = Eigen::Vector6d::Ones();
  std::string overwritten_Task;
  std::string arm_Task;
  std::string target_frame_;
  sva::PTransformd X_body_frame = sva::PTransformd::Identity();
  mc_rbdyn::FramePtr frame_;
  mc_rtc::Configuration config_;
  /// @}

  /// @{ State
  std::shared_ptr<mc_tasks::TransformTask> hand_taskTranslation;
  std::shared_ptr<mc_tasks::LookAtTask> hand_taskOrientation;
  std::shared_ptr<mc_tasks::VectorOrientationTask> hand_taskOrientationDown;
  double stiffness_ = 2.0;
  double weight_ = 0.0;
  double stiffnessOrientationTask_ = 2.0;
  double weightOrientationTask = 0.0;

  /// @{ 共享自主实验日志
  bool dataReady_ = false;
  bool selectedForAssistance_ = false;
  bool assistanceRequested_ = false;
  bool assistanceActive_ = false;
  bool translationAssistanceActive_ = false;
  bool orientationAssistanceActive_ = false;
  bool previousAssistanceActive_ = false;
  double effectiveAlpha_ = 0.0;
  double targetDistance_ = 0.0;
  double cumulativeActiveDuration_ = 0.0;
  double currentEpisodeDuration_ = 0.0;
  unsigned int activeEpisodeCount_ = 0;
  std::string currentGoalNameLog_ = "none";
  /// @}
  /// @}

};
