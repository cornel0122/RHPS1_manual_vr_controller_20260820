#pragma once

#include <mc_control/fsm/State.h>
#include <mc_filter/LowPass.h>
#include <mc_tasks/ForceConstrainedTransformTask.h>
#include <mc_tasks/OrientationTask.h>
#include "../ANAAvatarController.h"
#include "../ROSSubscriber.h"
#include "../motion_prediction/Motion_Prediction.h"
#include <mc_neuron_mocap_plugin/MoCapProperties.h>
#include <thread>

struct RelativeRetargetting : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void createGUI(mc_control::fsm::Controller & ctl);
  void createUnityGUI(mc_control::fsm::Controller & ctl);
  void createLogs(mc_control::fsm::Controller & ctl);
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
  void update_gui_trajectory(mc_control::fsm::Controller & ctl);
  void update_force_cstr(mc_control::fsm::Controller & ctl,
                         size_t id,
                         const sva::ForceVecd & admittance,
                         const sva::ForceVecd & force_lim,
                         const sva::ForceVecd & force_margin,
                         const sva::MotionVecd & damping);
  void motion_prediction_thread_loop();

private:
  /// @{ CONFIG
  std::string datastoreRetargetingReferenceRobot_ = "RetargetingReferenceRobot";
  std::string datastoreRetargetingReferenceOperator_ = "RetargetingReferenceOperator";
  MoCap_Body_part End_effect = MoCap_Body_part::LeftHand;

  std::string hand_topic_;
  std::string hand_vel_topic_;
  std::string reference_topic_;
  std::string state_topic_;
  std::string trigger_topic_;
  ros::Publisher retargetting_state_pub_;
  ros::Publisher state_trigger_pub_;
  ROSBoolSubscriber state_trigger_sub_;
  ROSPoseStampedSubscriber hand_pose_sub_;
  ROSAccelStampedSubscriber hand_vel_sub_;
  double maxTime_ = 0.5;
  bool use_vive_trackers_ = false;
  bool ignore_emergency_ = false;
  double scaling_ = 1.0;

  // For the origin for retargetting we have two choices:
  // - Use the robot head pose
  // - Use the occulus pose (from the datastore)
  std::string originFrame_ = "";
  sva::PTransformd originFrameOffset_ = sva::PTransformd::Identity();
  // Origin from datastore
  std::string datastoreHeadFrame_ = "";
  bool useDatastoreHeadFrame_ = false;

  std::string targetFrame_ = "";
  sva::PTransformd X_body_frame_ = sva::PTransformd::Identity();
  mc_rbdyn::FramePtr frame_;
  mc_rbdyn::FramePtr frame_zmp_;
  sva::PTransformd targetFrameOffset_ = sva::PTransformd::Identity();
  std::vector<std::string> activeJoints_; // If empty use all joints
  Eigen::Vector6d dimWeight_ = Eigen::Vector6d::Ones();
  Eigen::Vector6d dimStiffness_ = Eigen::Vector6d::Ones();
  mc_rtc::Configuration config_;
  /// @}

  /// @{ State
  std::shared_ptr<mc_tasks::ForceConstrainedTransformTask> hand_task_ = nullptr;
  double minStiffness_ = 5;
  double maxStiffness_ = 200;
  size_t hand_control_activatedTimestep_ = 0;
  size_t trigger_on_count_ = 0;
  double maxStiffTimeThreshold_ = 10; // Time after which hand task gain reach max [s]
  double linearStiffTimeThreshold_ = 5; // Time after which hand task gain switch from min to gradually reach max [s]
  double weight_ = 1000;
  double ori_weight_fact = 0.;
  ///@}

  /// @{ State

  sva::MotionVecd safety_damping_ = sva::MotionVecd::Zero();
  sva::ForceVecd admittance_ = sva::ForceVecd::Zero();
  sva::MotionVecd safety_damping_zmp_ = sva::MotionVecd::Zero();
  sva::ForceVecd admittance_zmp_ = sva::ForceVecd::Zero();
  sva::ForceVecd measured_wrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd measured_wrench_zmp_ = sva::ForceVecd::Zero();
  sva::ForceVecd force_margin_ = sva::ForceVecd::Zero();
  sva::ForceVecd force_limit_ = sva::ForceVecd::Zero();
  sva::ForceVecd force_margin_zmp_ = sva::ForceVecd::Zero();
  sva::ForceVecd force_limit_zmp_ = sva::ForceVecd::Zero();
  size_t upper_cstr_id_ = 0;
  size_t lower_cstr_id_ = 1;
  size_t cstr_id = 2;
  size_t upper_cstr_id_zmp_ = 3;
  size_t lower_cstr_id_zmp_ = 4;

  /// @}
  double softJumpSafetyDistance_ = 0.05;
  double criticalJumpSafetyDistance_ = 0.1;
  double maxHandDistanceFromReferenceSafety_ = 2.0;
  double maxElbowDistanceFromChestSafety_ = 1.5;
  bool safety_active_ = false;

  // Motion Prediction Sequence parameters
  std::thread Prediction_Computation_thread;
  bool thread_compute_trigger = false;
  bool thread_on = false;
  Motion_Prediction target_Frame_prediction_;
  Eigen::MatrixXd PoseSeq;
  Eigen::MatrixXd AccSeq;
  sva::PTransformd predicted_target_ = sva::PTransformd::Identity();
  std::string data_sequence_name = "";
  int data_frequency_ = 60;
  int model_frequency_ = 30;

  int input_sequence_length_ = 15;
  int output_sequence_length_ = 15;
  double t_forward_ = 0.1;
  bool Prediction_On_ = false;
  bool Compute_Prediction_ = false;
  Eigen::Vector3d T_U_hand_pred = Eigen::Vector3d::Zero();
  Eigen::Vector3d V_mw_hand_pred = Eigen::Vector3d::Zero();
  sva::PTransformd X_chest_hand_pred_ = sva::PTransformd::Identity();
  Eigen::Vector3d V_chest_hand_lin = Eigen::Vector3d::Zero();
  mc_filter::LowPass<Eigen::Vector6d> vel_filter = mc_filter::LowPass<Eigen::Vector6d>(5e-3);
  mc_filter::LowPass<Eigen::Vector3d> vel_pred_filter = mc_filter::LowPass<Eigen::Vector3d>(5e-3);
  Eigen::Vector6d vel_target_;
  Eigen::Vector3d lin_vel_pred_target_;
  std::vector<Eigen::Vector3d> Pose_Seq_ = {};
  std::vector<Eigen::Vector3d> Predicted_Pose_Seq_ = {};
  std::chrono::high_resolution_clock::time_point
      t_controller_prediction; // Use to update the prediction at the data sequence frequency

  bool active_ = false; // Whether the pose from ROS should be tracked
  bool data_online_ = false;
  bool user_active_ = false;
  bool unity_online_ = false;

  bool active_if_possible_ = false;
  bool first_active_ = true; // Set to false once the retargetting is active for the first time;
  sva::PTransformd X_u0_pose_ = sva::PTransformd::Identity(); // Hand pose from Unity
  sva::MotionVecd V_pose_ = sva::MotionVecd::Zero(); // Hand velocity at hand pose in global frame
  bool trigger_ = false;

  sva::PTransformd X_chest_hand_ = sva::PTransformd::Identity();
  sva::PTransformd target_ = sva::PTransformd::Identity(); // Target pose (only used if active_=true)
  sva::MotionVecd ref_vel_ = sva::MotionVecd::Zero();
  Eigen::Matrix3d arm_target_ = Eigen::Matrix3d::Identity();

  std::shared_ptr<ros::NodeHandle> nh_;

  mc_tasks::PostureTaskPtr posture_override_;
  bool posture_override_active_ = false;
};
