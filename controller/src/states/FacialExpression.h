#pragma once

#include <mc_control/fsm/State.h>
#include "../ROSSubscriber.h"

struct FacialExpression : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void setJointTargetPercent(mc_control::fsm::Controller & ctl, const std::string & jnt_name, double tar);
  void setNominal(mc_control::fsm::Controller & ctl);
  void createGUI(mc_control::fsm::Controller & ctl);

private:
  /// @{ CONFIG
  std::string sub_topic_ = "avatar/face_data";
  Eigen::VectorXd stiffnesses_ = Eigen::Vector4d::Ones();
  /**
   * Active joints are expected to be provided in the same order as FaceJoints
   */
  std::vector<std::string> activeJoints_ = {};
  bool Has_face_joints = true;
  ///@}
  std::shared_ptr<ros::NodeHandle> nh_;
  ROSMultiArraySubscriber face_sub_;
  bool face_data_online_ = false;
  std::vector<double> face_data_;
  double maxTime_ = 0.5;
  /// @{ State
  std::shared_ptr<mc_tasks::PostureTask> postureTask_ = nullptr;

  /**
   * @brief Defines the reference order of the face joints
   * It is expected that the subscribed data follows this order.
   *
   * Do not reorder these elements unless the subscribed data order changes, in which case also make sure activeJoints_
   * order is updated accordingly.
   */
  enum FaceJoints
  {
    EYE_Y = 0,
    EYE_P,
    EYELID_P,
    MOUTH_P,
    COUNT = MOUTH_P + 1
  };
  size_t jindex(FaceJoints joint) const noexcept { return static_cast<size_t>(joint); };
  const std::string & jname(FaceJoints joint) const { return activeJoints_[jindex(joint)]; };

  int ticks_ = 0; // counter
  double period_ = 0.5; // in seconds
  double interval_ = 5; // in seconds
  ///@}

  bool active_ = true;
  bool size_warning_on_ = false;
};
