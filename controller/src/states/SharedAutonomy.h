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
