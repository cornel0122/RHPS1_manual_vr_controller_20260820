#include <mc_control/fsm/State.h>
#include "../ROSSubscriber.h"
#include <mc_neuron_mocap_plugin/MoCapProperties.h>

struct SynchroWalk : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void activate();
  void deactivate();
  bool active() { return active_ || active_trigger_mode_; }
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void node_sub(ros::NodeHandle nh);

  void tick(double dt)
  {
    sub_Ts.tick(dt);
    sub_Tds.tick(dt);
    sub_SupportState.tick(dt);
    sub_StepId.tick(dt);
    sub_SupportFoot.tick(dt);
    sub_Stop.tick(dt);
    sub_Ratio.tick(dt);
    state_trigger_sub_.tick(dt);
  }

  void Update_values()
  {

    PredictedTs = sub_Ts.data().value();
    PredictedTds = sub_Tds.data().value();
    FeetUp = sub_SupportState.data().value();
    User_Step_ID = sub_StepId.data().value();
    SupportFoot = sub_SupportFoot.data().value();
    stop = sub_Stop.data().value();
    Ratio = sub_Ratio.data().value();
  }

  bool MatchingStep() { return User_Step_ID == Robot_Step_ID; }

  double get_Tds() { return PredictedTds; }
  double get_Ts() { return PredictedTs; }

  bool UserInSingleSupport() { return FeetUp; }

  void update_Prev_values(double val);
  bool New_Values();
  bool Stop();

  bool Datas_Online()
  {
    if(!trigger_walk_mode_) { return sub_Ts.data().isValid(); }

    if(use_vive_trackers) { return left_leg_online_ || right_leg_online_; }
    else { return mocap_online_; }
  }

  void User_Lifted(double t);
  void Robot_Lifted(double t);
  void User_Stepped(double t);
  void Robot_Stepped(double t);
  void reset_checkpoints();
  bool IsStepDone();

  // Predicted Timing at computation time (i.e : Time remaining)
  double PredictedTs = 0.0;
  double PredictedTds = 0.0;
  double Prev_Pred_Tds = 0.0;
  double Prev_Pred_Ts = 0.0;
  double t_pred = 0;
  int stop = 1;

  double UsedTs = 0.0;
  double UsedTds = 0.0;

  double Ratio = 0.0;

  std::string SupportFoot;

  int User_Step_ID = 0;
  int Robot_Step_ID = 0;
  int Vel_ID = 0;

private:
  Eigen::Vector3d arrow_start_;
  Eigen::Vector3d arrow_end_;
  sva::PTransformd X_m0_Hips = sva::PTransformd::Identity();
  Eigen::Vector3d Vlin_Hips_Hips;
  sva::PTransformd hips_pose_ = sva::PTransformd::Identity();
  Eigen::Vector3d input_vel_ = Eigen::Vector3d::Zero();
  MoCap_Body_part swing_foot = LeftFoot;

  sva::MotionVecd left_leg_vel_ = sva::MotionVecd::Zero();
  sva::MotionVecd right_leg_vel_ = sva::MotionVecd::Zero();
  sva::MotionVecd prev_left_leg_vel_ = sva::MotionVecd::Zero();
  sva::MotionVecd prev_right_leg_vel_ = sva::MotionVecd::Zero();
  bool left_leg_online_ = false;

  bool right_leg_online_ = false;
  bool mocap_online_ = false;
  bool use_vive_trackers = true;
  bool useROS = false;

  bool active_ = false;
  bool active_trigger_mode_ = false;
  bool trigger_walk_mode_ = true;
  double activation_threshold = 2;
  size_t robot_walk_count = 0; // used only in trigger mode

  std::shared_ptr<ros::NodeHandle> nh_;

  std::string next_ts_topic_ = "avatar/NextTs";
  std::string next_tds_topic_ = "avatar/NextTds";
  std::string step_id_topic_ = "avatar/StepID";
  std::string stop_topic_ = "avatar/Stop";
  std::string support_foot_topic_ = "avatar/SupportFoot";
  std::string single_support_topic_ = "avatar/FeetUp";
  std::string ratio_topic_ = "avatar/Ratio";
  std::string datastoreRetargetingReference_ = "";

  std::string state_topic_;
  std::string trigger_topic_;

  size_t trigger_on_count_ = 0;

  ros::Publisher walking_state_pub_;
  ros::Publisher state_trigger_pub_;
  ROSBoolSubscriber state_trigger_sub_;

  ROSFloatSubscriber sub_Ts;
  ROSFloatSubscriber sub_Tds;
  ROSIntSubscriber sub_SupportState;
  ROSIntSubscriber sub_StepId;
  ROSStringSubscriber sub_SupportFoot;
  ROSIntSubscriber sub_Stop;
  ROSFloatSubscriber sub_Ratio;

  int FeetUp = 0; // For offline reading
  bool User_Foot_Contact = true; // Both user feets are on the ground

  double t_lift = 0.; // Time when the swing foot contact has been removed
  double t_lift_user = 0.; // Time when the user lift a foot
  double t_down = 0.; // Time when the swing foot contact has been added
  double t_down_user = 0.; // Time when the user ends his single support phase

  double maxTime_ = 0.5;

  bool trigger_ = false;

  int PrevFootState = 0;

  bool UserStepped = false;
  bool RobotStepped = false;
};
