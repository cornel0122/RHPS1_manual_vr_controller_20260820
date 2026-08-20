#include "SynchroWalk.h"

#include "../ANAAvatarController.h"

#include <mc_rtc/io_utils.h>

void SynchroWalk::configure(const mc_rtc::Configuration & config)
{
  config("next_ts_topic", next_ts_topic_);
  config("next_tds_topic", next_tds_topic_);
  config("stop_topic", stop_topic_);
  config("support_foot_topic", support_foot_topic_);
  config("single_support_topic", single_support_topic_);
  config("step_id_topic", step_id_topic_);
  config("ratio_topic", ratio_topic_);
  config("datastoreRetargetingReference", datastoreRetargetingReference_);
  config("state_ros_topic", state_topic_);
  config("trigger_ros_topic", trigger_topic_);
  config("trigger_mode", trigger_walk_mode_);
  config("activation_threshold", activation_threshold);
  config("use_vive_trakcer", use_vive_trackers);
}

void SynchroWalk::start(mc_control::fsm::Controller & ctl)
{
  nh_ = ana_ros_node_handle();
  sub_Ts.subscribe(*nh_, next_ts_topic_);
  sub_Ts.maxTime(maxTime_);
  sub_Tds.subscribe(*nh_, next_tds_topic_);
  sub_Tds.maxTime(maxTime_);
  sub_Stop.subscribe(*nh_, stop_topic_);
  sub_Stop.maxTime(maxTime_);
  sub_SupportFoot.subscribe(*nh_, support_foot_topic_);
  sub_SupportFoot.maxTime(maxTime_);
  sub_SupportState.subscribe(*nh_, single_support_topic_);
  sub_SupportState.maxTime(maxTime_);
  sub_Ratio.subscribe(*nh_, ratio_topic_);
  sub_Ratio.maxTime(maxTime_);
  sub_StepId.subscribe(*nh_, step_id_topic_);
  sub_StepId.maxTime(maxTime_);

  if(ctl.datastore().get<bool>("UseROS"))
  {
    useROS = true;
    state_trigger_sub_.subscribe(*nh_, trigger_topic_);
    state_trigger_sub_.maxTime(maxTime_);
    walking_state_pub_ = nh_->advertise<std_msgs::Bool>(state_topic_, 1);
    state_trigger_pub_ = nh_->advertise<std_msgs::Bool>(trigger_topic_, 1);
  }

  arrow_start_ = ctl.robot().com();
  arrow_end_ = ctl.robot().com();

  ctl.datastore().make_call("synchro_walk::deactivate",
                            [this]()
                            {
                              active_ = false;
                              mc_rtc::log::info("[{}] Synchro walk off", name());
                            });
  ctl.datastore().make_call("synchro_walk::active", [this]() -> bool { return active_ || active_trigger_mode_; });
  ctl.datastore().make_call("synchro_walk::on_off",
                            [this]()
                            {
                              active_ = !active_;
                              if(active_) { mc_rtc::log::info("[{}] Synchro walk active", name()); }
                              else { mc_rtc::log::info("[{}] Synchro walk off", name()); }
                            });
  auto & gui = *ctl.gui();

  gui.addElement(
      {"Avatar"}, mc_rtc::gui::Label("Step synchro online", [this]() -> const bool { return Datas_Online(); }),
      // mc_rtc::gui::Arrow(
      //     "Walking dir", [this]() { return arrow_start_; }, [this]() { return arrow_end_; }),
      mc_rtc::gui::Transform("Hips frame", [this]() -> sva::PTransformd { return hips_pose_; }),
      mc_rtc::gui::Checkbox(
          "Synchronise user's step", [this]() { return active_ || active_trigger_mode_; }, [this]() { activate(); })

  );

  gui.addElement(this, {"Avatar", "Unity", "Trigger"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return trigger_; }, [this]() { trigger_ = !trigger_; }));
  gui.addElement(this, {"Avatar", "Unity", "State"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> bool { return active(); }, [this]() {}));
  gui.addElement(this, {"Avatar", "Unity", "Walking"},
                 mc_rtc::gui::ArrayInput(
                     name() + "Left", [this]() -> sva::MotionVecd { return left_leg_vel_; },
                     [this](sva::MotionVecd in) { left_leg_vel_ = in; }));
  gui.addElement(this, {"Avatar", "Unity", "Walking"},
                 mc_rtc::gui::Checkbox(
                     name() + "LeftOnline", [this]() -> bool { return left_leg_online_; },
                     [this]() { left_leg_online_ = !left_leg_online_; }));
  gui.addElement(this, {"Avatar", "Unity", "Walking"},
                 mc_rtc::gui::ArrayInput(
                     name() + "Right", [this]() -> sva::MotionVecd { return right_leg_vel_; },
                     [this](sva::MotionVecd in) { right_leg_vel_ = in; }));
  gui.addElement(this, {"Avatar", "Unity", "Walking"},
                 mc_rtc::gui::Checkbox(
                     name() + "RightOnline", [this]() -> bool { return right_leg_online_; },
                     [this]() { right_leg_online_ = !right_leg_online_; }));
}

void SynchroWalk::activate()
{
  if(!trigger_walk_mode_) { active_ = !active_; }
  else { active_trigger_mode_ = !active_trigger_mode_; }
}
void SynchroWalk::deactivate()
{
  active_ = false;
  active_trigger_mode_ = false;
}

bool SynchroWalk::run(mc_control::fsm::Controller & ctl)
{

  if(!ctl.datastore().has("mocap_plugin::online"))
  {
    output("OK");
    return true;
  }

  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  auto robot_walking_func = [&]() { return walking.is_walking(); };
  auto robot_stop_phase_func = [&]() { return walking.is_stopping() || walking.is_stopped(); };
  auto robot_stop_start_func = [&]() { return walking.start_stop_walking(); };
  auto double_support_func = [&]() { return walking.is_double_support(); };
  auto support_foot_func = [&]() { return walking.get_support_foot(); };
  // FIXME We don't use them for now
  auto t_func = []() { return 0.0; };
  auto t_lift_func = []() { return 0.0; };
  auto t_contact_func = []() { return 0.0; };
  auto input_ts_func = [](double) {};
  auto input_tds_func = [](double) {};
  auto n_steps_func = [](int) {};
  auto tds_by_ratio_func = [](bool) {};

  auto & mocap_pose_func =
      ctl.datastore().get<std::function<sva::PTransformd(MoCap_Body_part)>>("mocap_plugin::get_pose");
  auto & mocap_vel_func =
      ctl.datastore().get<std::function<sva::MotionVecd(MoCap_Body_part)>>("mocap_plugin::get_velocity");
  auto & mocap_acc_func =
      ctl.datastore().get<std::function<Eigen::Vector3d(MoCap_Body_part)>>("mocap_plugin::get_accel");
  auto & mocap_freq_func = ctl.datastore().get<std::function<int(void)>>("mocap_plugin::get_data_frequency");
  mocap_online_ = ctl.datastore().get<bool>("mocap_plugin::online");

  tick(ctl.timeStep);
  bool trigger = trigger_;
  if(ctl.datastore().get<bool>("UseROS")) { trigger = state_trigger_sub_.data().value(); }

  if(trigger && trigger_on_count_ * ctl.timeStep > 1)
  {
    activate();
    trigger_ = false;
    if(ctl.datastore().get<bool>("UseROS"))
    {
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      trigger_on_count_ = 0;
      state_trigger_pub_.publish(trigger_msg);
    }
  }
  trigger_on_count_++;

  if(Datas_Online() && !trigger_walk_mode_)
  {
    Update_values();
    arrow_start_ = ctl.robot().com();
    X_m0_Hips = mocap_pose_func(Hips);
    hips_pose_ = sva::PTransformd(X_m0_Hips.rotation(), arrow_start_);
    sva::MotionVecd V_Hips_m0 = mocap_vel_func(Hips);
    sva::MotionVecd V_Rf_m0 = mocap_vel_func(RightFoot);
    Eigen::Vector3d V_Rf_hips = X_m0_Hips.rotation() * V_Rf_m0.linear();

    Vlin_Hips_Hips = X_m0_Hips.rotation() * V_Hips_m0.linear();
    // arrow_end_ = arrow_start_ + Eigen::Vector3d{(X_0_Hips.rotation() * V_Hips_0.linear()).z(), 0, 0};
    arrow_end_ = arrow_start_ + V_Rf_hips;
  }

  if(active() && !Datas_Online())
  {
    mc_rtc::log::warning("[{}] Data Offline, deactivating ", name());
    deactivate();
  }
  if(active() && ctl.datastore().get<bool>("Emergency"))
  {
    mc_rtc::log::warning("[{}] Emergency triggered, deactivating ", name());
    deactivate();
    if(!robot_stop_phase_func()) { robot_stop_start_func(); }
  }
  if(active()
     && (ctl.datastore().get<bool>("ANA::ComTracking") || !ctl.datastore().get<bool>("ANA::ComTracking::CoMUp")))
  {
    mc_rtc::log::warning("[{}] Robot is not in walking pose, deactivating ", name());
    deactivate();
  }
  if(active() && ctl.datastore().get<bool>("Emergency"))
  {
    mc_rtc::log::warning("[{}] Emergency state ", name());
    deactivate();
  }
  if(ctl.datastore().get<bool>("Emergency"))
  {
    if(!robot_stop_phase_func()) { robot_stop_start_func(); }
  }

  if(active_)
  {
    // mc_rtc::log::info(stop);

    n_steps_func(-1);
    tds_by_ratio_func(false);
    double tds = get_Tds();
    double ts = get_Ts();
    int id = User_Step_ID;

    bool NewVal_Tds = tds != Prev_Pred_Tds;
    bool NewVal_Ts = ts != Prev_Pred_Ts;

    if(t_lift == 0 && !double_support_func())
    {
      mc_rtc::log::info("ROBOT UP");
      Robot_Lifted(t_lift_func());
    }

    if(double_support_func() && !RobotStepped)
    {
      mc_rtc::log::info("ROBOT DOWN");
      Robot_Stepped(t_contact_func());
    }

    if(User_Foot_Contact && (UserInSingleSupport()))
    {
      mc_rtc::log::info("USER UP");
      User_Lifted(t_func());
    }
    if(!User_Foot_Contact && (!UserInSingleSupport()))
    {
      mc_rtc::log::info("USER DOWN");
      User_Stepped(t_func());
    }

    IsStepDone();
    User_Foot_Contact = (!UserInSingleSupport());

    if(stop == 0)
    {
      // mc_rtc::log::info("[{}] input vel \n{}",name(),input_vel_);
      if(double_support_func() && !MatchingStep())
      {
        // if (!robot_walking_func())
        // {
        input_vel_ = Eigen::Vector3d::Zero();
        // }
        input_ts_func(ts);

        input_tds_func(tds);
        if(robot_stop_phase_func()) { robot_stop_start_func(); }

        Robot_Step_ID = id;

        // if(std::abs(Vlin_Hips_Hips.z()) > 0.02)
        // {
        //   input_vel_.x() = Vlin_Hips_Hips.z();
        // }
        // if(std::abs(Vlin_Hips_Hips.y()) > 0.05)
        // {
        //   input_vel_.y() = -Vlin_Hips_Hips.y();
        // }
        // set_vel_func(input_vel_);

        mc_rtc::log::info("GO at ID {} for Tds {} and Ts {}", id, tds, ts);
        // if(SupportFoot != support_foot_func() && !robot_walking_func())
        // {
        //   switch_suport_foot_func();
        // }
        swing_foot = LeftFoot;
        if(support_foot_func() == "LeftFoot") { swing_foot = RightFoot; }

        Prev_Pred_Tds = tds;
        Prev_Pred_Ts = ts;
      }
    }
    if(NewVal_Ts && !double_support_func() && walking.next_ts() - t_func() > 0.2 && std::abs(Prev_Pred_Ts - ts) < 0.15
       && MatchingStep())
    {

      Prev_Pred_Ts = ts;
      input_ts_func(ts);
    }
    sva::MotionVecd V_swing_m0 = mocap_vel_func(swing_foot);
    Eigen::Vector3d Vlin_swing_Hips = X_m0_Hips.rotation() * V_swing_m0.linear();
    if(stop == 1 && !robot_stop_phase_func()) // && double_support_func())
    {
      mc_rtc::log::info("STOP");
      robot_stop_start_func();
    }
  }

  else if(active_trigger_mode_)
  {
    Eigen::Vector3d left_metric;
    Eigen::Vector3d right_metric;
    if(!use_vive_trackers)
    {
      left_metric = mocap_acc_func(LeftFoot);
      right_metric = mocap_acc_func(RightFoot);
      // mc_rtc::log::info("[{}] left {} ; right {}",name(),left_metric.z(),right_metric.z() );
    }
    else
    {
      left_metric = right_leg_vel_.linear();
      right_metric = left_leg_vel_.linear();
    }
    if((fabs(left_metric.z()) > activation_threshold || fabs(right_metric.z()) > activation_threshold))
    {
      robot_walk_count = 0;
      if(robot_stop_phase_func()) { robot_stop_start_func(); }
    }
    if(!robot_stop_phase_func())
    {
      robot_walk_count++;
      if(robot_walk_count * ctl.timeStep > 0.9 * walking.next_ts()) { robot_stop_start_func(); }
    }
  }

  else
  {
    tds_by_ratio_func(true);
    // if(!robot_stop_phase_func())
    // {
    //   robot_stop_start_func();
    // }
  }

  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Bool state_msg;
    state_msg.data = active_trigger_mode_ || active_;
    walking_state_pub_.publish(state_msg);
  }

  output("OK");
  return true;
}

void SynchroWalk::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

void SynchroWalk::User_Stepped(double t)
{
  t_down_user = t;
  mc_rtc::log::success("t_user_down : {} ", t);
  UserStepped = true;
}
void SynchroWalk::Robot_Stepped(double t)
{
  t_down = t;
  mc_rtc::log::success("t_down {} : ", t);
  RobotStepped = true;
}
void SynchroWalk::User_Lifted(double t)
{
  t_lift_user = t;
  mc_rtc::log::success("t_lift_user {} : ", t);
}
void SynchroWalk::Robot_Lifted(double t)
{
  t_lift = t;
  mc_rtc::log::success("t_lift {} : ", t);
}

void SynchroWalk::reset_checkpoints()
{
  RobotStepped = false;
  UserStepped = false;
  t_down_user = 0.;
  t_lift = 0.;
  t_lift_user = 0.;
  t_down = 0.;
}

bool SynchroWalk::IsStepDone()
{
  bool out = RobotStepped && UserStepped;
  if(out)
  {
    RobotStepped = false;
    UserStepped = false;
    mc_rtc::log::success("Delta Tss : {} ", std::abs(t_down - t_down_user));
    mc_rtc::log::success("Delta Tds : {} ", std::abs(t_lift - t_lift_user));
    reset_checkpoints();
  }
  // else if (std::abs(t_lift - t_lift_user) > 0.5)
  // {
  //   mc_rtc::log::info("reset counter");
  //   reset_checkpoints();
  //   out = true;
  // }

  return out;
}

EXPORT_SINGLE_STATE("SynchroWalk", SynchroWalk)
