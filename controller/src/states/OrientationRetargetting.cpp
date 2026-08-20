#include "OrientationRetargetting.h"
// #include "../ROSSubscriber.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>

void OrientationRetargetting::configure(const mc_rtc::Configuration & config)
{

  config("datastoreRetargetingReferenceRobot", datastoreRetargetingReferenceRobot_);
  config("datastoreRetargetingReferenceOperator", datastoreRetargetingReferenceOperator_);
  config("dimWeight", dimWeight_);
  config("dimStiffness", dimStiffness_);
  config("minStiffness", minStiffness_);
  config("maxStiffness", maxStiffness_);
  config("weight", weight_);
  config("linearStiffTimeThreshold", linearStiffTimeThreshold_);
  config("maxStiffTimeThreshold", maxStiffTimeThreshold_);

  config("criticalJumpSafetyDistance", criticalJumpSafetyDistance_);

  config("vive_trackers", use_vive_trackers_);
  config("body_ros_topic", body_topic_);
  config("body_vel_ros_topic", body_vel_topic_);
  config("reference_ros_topic", reference_topic_);
  config("always_actve", active_if_possible_);

  config("state_ros_topic", state_topic_);
  config("trigger_ros_topic", trigger_topic_);

  config_.load(config);
}

void OrientationRetargetting::start(mc_control::fsm::Controller & ctl)
{
  if(!config_.has("robot") || (config_.has("robot") && !config_("robot").has(ctl.robot().name())))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot {} defined in the YAML configuration", name(),
                                                     ctl.robot().name());
  }
  auto rConfig = config_("robot")(ctl.robot().name());
  rConfig("target_frame", target_frame_);
  rConfig("activeJoints", activeJoints_);
  rConfig("body_frame_relative_pose", X_body_frame);

  auto & mocap_freq_func = ctl.datastore().get<std::function<int(void)>>("mocap_plugin::get_data_frequency");

  int part = 0;
  config_("mocap_end_eff_num", part);
  std::cout << name() << " End eff num " << part << std::endl;
  config_("mocap_ori_body_num", part);
  // ori_task_body = part;
  ori_task_body = static_cast<MoCap_Body_part>(part);

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();

    if(use_vive_trackers_)
    {
      body_pose_sub_.subscribe(*nh_, body_topic_);
      body_pose_sub_.maxTime(maxTime_);

      vel_sub_.subscribe(*nh_, body_vel_topic_);
      vel_sub_.maxTime(maxTime_);
    }
    state_trigger_sub_.subscribe(*nh_, trigger_topic_);
    state_trigger_sub_.maxTime(maxTime_);
    retargetting_state_pub_ = nh_->advertise<std_msgs::Bool>(state_topic_, 1);
    state_trigger_pub_ = nh_->advertise<std_msgs::Bool>(trigger_topic_, 1);
  }

  ctl.datastore().make_call(name() + "::toggle_walking_mode",
                            [this]()
                            {
                              if(body_task_->weight() != 0) { body_task_->weight(0); }
                              else { body_task_->weight(weight_); }
                            });
  ctl.datastore().make_call(name() + "::active", [this]() -> bool { return active_; });
  ctl.datastore().make_call(name() + "::data_online", [this]() -> bool { return data_online_; });
  ctl.datastore().make_call(name() + "::activate_deactivate",
                            [this, &ctl]()
                            {
                              if(!active_) { user_activate(ctl); }
                              else { user_deactivate(ctl); }
                            });

  frame_ = mc_rbdyn::RobotFrame::make(name() + "ControlFrame", ctl.robot().frame(target_frame_), X_body_frame, false);

  body_task_ =
      std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(name() + "ControlFrame"), minStiffness_, weight_);
  body_task_->reset();
  body_task_->dimWeight(dimWeight_);
  R_0_body_target_ = body_task_->frame().position().rotation();

  ctl.datastore().make<std::shared_ptr<mc_tasks::TransformTask>>(name() + "Task", body_task_);

  if(activeJoints_.size())
  {
    mc_rtc::log::info("[{}] Active joints: [{}]", name(), mc_rtc::io::to_string(activeJoints_));
    body_task_->selectActiveJoints(activeJoints_);
  }

  Eigen::Vector6d dof = Eigen::Vector6d::Zero();
  dof << 1, 1, 1, 1, 1, 1;

  ctl.solver().addTask(body_task_);

  const auto X_0_chest_reference = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceRobot_);
  const auto X_0_body = ctl.robot().frame(target_frame_).position();

  X_chest_body_ = X_0_body * X_0_chest_reference.inv();

  createGUI(ctl);
  createUnityGUI(ctl);

  auto & logger = ctl.logger();

  MC_RTC_LOG_HELPER(name() + "_retargetting_active", active_);
}

void OrientationRetargetting::activate(mc_control::fsm::Controller & ctl)
{

  if(!active_ && data_online_ && !(ctl.datastore().get<bool>("Emergency")))
  {
    mc_rtc::log::info("[{}] Activated tracking for {}", name(), body_task_->name());
    if(first_active_)
    {
      // avoid having the offsets preloaded before retrieving data
      auto rConfig = config_("robot")(ctl.robot().name());
      rConfig("originOffset", originFrameOffset_);
      rConfig("targetOffset", targetFrameOffset_);
      first_active_ = false;
    }
    body_control_activatedTimestep_ = 0;
    body_task_->reset();
    active_ = true;
  }
  else { mc_rtc::log::warning("[{}] Failed to activate, data status {}", name(), data_online_); }
}

void OrientationRetargetting::deactivate(mc_control::fsm::Controller & ctl)
{

  if(active_) // If the task was active, reset the target to the current pose once
  {
    mc_rtc::log::info("[{}] Deactivated tracking for {}", name(), body_task_->name());
  }
  body_task_->stiffness(minStiffness_);
  active_ = false;
  body_control_activatedTimestep_ = 0;
}

bool OrientationRetargetting::run(mc_control::fsm::Controller & ctl)
{
  // Automatically activate if the index trigger button is pressed
  if(ctl.datastore().get<bool>("Emergency")) { user_deactivate(ctl); }
  if(!ctl.datastore().has("mocap_plugin::online"))
  {
    output("OK");
    return true;
  }

  auto & mocap_pose_func =
      ctl.datastore().get<std::function<sva::PTransformd(MoCap_Body_part)>>("mocap_plugin::get_pose");
  auto & mocap_vel_func =
      ctl.datastore().get<std::function<sva::MotionVecd(MoCap_Body_part)>>("mocap_plugin::get_velocity");
  auto & mocap_online = ctl.datastore().get<bool>("mocap_plugin::online");

  data_online_ = mocap_online;

  const auto X_0_chest_reference = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceRobot_);
  auto ori = mocap_pose_func(ori_task_body);
  auto oriVel = mocap_vel_func(ori_task_body);
  bool trigger = trigger_;

  if(ctl.datastore().get<bool>("UseROS"))
  {
    state_trigger_sub_.tick(ctl.solver().dt());
    trigger = state_trigger_sub_.data().value();
  }

  if(use_vive_trackers_)
  {
    data_online_ = unity_online_ && ctl.datastore().get<bool>(datastoreRetargetingReferenceOperator_ + "Active");
    ori = X_u0_body_;
    oriVel = V_body_;
    if(ctl.datastore().get<bool>("UseROS"))
    {
      body_pose_sub_.tick(ctl.solver().dt());
      vel_sub_.tick(ctl.solver().dt());
      ori = body_pose_sub_.data().value();
      oriVel = vel_sub_.data().value();
      oriVel = sva::MotionVecd(-oriVel.angular(), oriVel.linear());
      data_online_ = body_pose_sub_.data().isValid();
    }
  }

  if(trigger && trigger_on_count_ * ctl.timeStep > 1)
  {
    if(active_) { user_deactivate(ctl); }
    else { user_activate(ctl); }
    if(ctl.datastore().get<bool>("UseROS"))
    {
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      state_trigger_pub_.publish(trigger_msg);
    }
    trigger_ = false;
    trigger_on_count_ = 0;
  }
  trigger_on_count_++;

  if(data_online_ && active_if_possible_ && !first_active_ && !active_ && user_active_) { activate(ctl); }

  if(active_ && data_online_)
  {

    auto X_m0_body = ori;
    auto X_m0_chest = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceOperator_);
    auto V_body_m0 = oriVel;

    sva::MotionVecd V_body_chest = sva::PTransformd(X_m0_chest.rotation()) * V_body_m0;
    sva::MotionVecd V_body_0 = sva::PTransformd(originFrameOffset_.rotation()).inv() * V_body_chest;

    ref_vel_ = sva::PTransformd(body_task_->frame().position().rotation()) * V_body_0;

    X_chest_body_ = X_m0_body * X_m0_chest.inv();

    if(body_control_activatedTimestep_ * ctl.timeStep <= maxStiffTimeThreshold_) { body_control_activatedTimestep_++; }

    if(body_control_activatedTimestep_ * ctl.timeStep < linearStiffTimeThreshold_)
    {
      body_task_->stiffness(minStiffness_ * dimStiffness_);
      // mc_rtc::log::info("[{}] min",name());
    }
    else
    {
      double ratio = (body_control_activatedTimestep_ * ctl.timeStep - linearStiffTimeThreshold_)
                     / (maxStiffTimeThreshold_ - linearStiffTimeThreshold_);
      mc_filter::utils::clampInPlace(ratio, 0, 1);
      body_task_->stiffness(minStiffness_ + ratio * (maxStiffness_ - minStiffness_));
      // mc_rtc::log::info("[{}] increasing",name());
    }
  }
  else
  {
    ref_vel_ = sva::MotionVecd::Zero();
    deactivate(ctl);
  }
  R_0_body_target_ = (targetFrameOffset_ * X_chest_body_ * originFrameOffset_ * X_0_chest_reference).rotation();
  body_task_->target(sva::PTransformd(R_0_body_target_, body_task_->frame().position().translation()));
  body_task_->refVelB(sva::MotionVecd(ref_vel_.angular(), Eigen::Vector3d::Zero()));

  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Bool state_msg;
    state_msg.data = active_;
    retargetting_state_pub_.publish(state_msg);
  }

  output("OK");
  return true;
}

void OrientationRetargetting::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
}

void OrientationRetargetting::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  auto & mocap_online = ctl.datastore().get<bool>("mocap_plugin::online");

  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Label(name() + "Data Online", [this]() -> const bool & { return data_online_; }));

  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; },
                     [this, &ctl]()
                     {
                       if(!active_) { activate(ctl); }
                       else { deactivate(ctl); }
                     }));
  gui.addElement(this, {"Avatar", name(), "Task"},
                 mc_rtc::gui::NumberInput(
                     "Min stiffness", [this]() { return minStiffness_; }, [this](double s) { minStiffness_ = s; }),
                 mc_rtc::gui::NumberInput(
                     "Max stiffness", [this]() { return maxStiffness_; }, [this](double s) { maxStiffness_ = s; }),
                 mc_rtc::gui::ArrayInput(
                     "Dim Stiffness", {"r", "p", "y", "x", "y", "z"},
                     [this]() -> const Eigen::Vector6d & { return dimStiffness_; },
                     [this](const Eigen::Vector6d & t) { dimStiffness_ = t; }),
                 mc_rtc::gui::NumberInput(
                     "Weight", [this]() { return body_task_->weight(); }, [this](double w) { body_task_->weight(w); }));

  gui.addElement(this, {"Avatar", name(), "Offsets"},
                 mc_rtc::gui::ArrayInput(
                     "Origin offset (translation) [m]", {"x", "y", "z"},
                     [this]() -> const Eigen::Vector3d & { return originFrameOffset_.translation(); },
                     [this](const Eigen::Vector3d & t) { originFrameOffset_.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "Origin offset (rotation) [deg]", {"r", "p", "y"},
                     [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(originFrameOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                     [this](const Eigen::Vector3d & rpy)
                     { originFrameOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));

  gui.addElement(this, {"Avatar", name(), "Offsets", "Target"},
                 mc_rtc::gui::ArrayInput(
                     "Target offset (translation) [m]", {"x", "y", "z"},
                     [this]() -> const Eigen::Vector3d & { return targetFrameOffset_.translation(); },
                     [this](const Eigen::Vector3d & t) { targetFrameOffset_.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "Target offset (rotation) [deg]", {"r", "p", "y"},
                     [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(targetFrameOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                     [this](const Eigen::Vector3d & rpy)
                     { targetFrameOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));
}

void OrientationRetargetting::createUnityGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();

  gui.addElement(this, {"Avatar", "Unity", "Pose"},
                 mc_rtc::gui::Transform(
                     name(), [this]() -> const sva::PTransformd & { return X_u0_body_; },
                     [this](const sva::PTransformd & pose) { X_u0_body_ = pose; }));
  gui.addElement(this, {"Avatar", "Unity", "Velocity"},
                 mc_rtc::gui::ArrayInput(
                     name(), {"wx", "wy", "wz", "vx", "vy", "vz"},
                     [this]() -> const sva::MotionVecd & { return V_body_; },
                     [this](sva::MotionVecd v) { V_body_ = v; }));
  gui.addElement(this, {"Avatar", "Unity", "Trigger"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return trigger_; }, [this]() { trigger_ = !trigger_; }));
  gui.addElement(this, {"Avatar", "Unity", "State"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return active_; }, [this]() {}));
  gui.addElement(
      this, {"Avatar", "Unity", "Data State"},
      mc_rtc::gui::Checkbox(
          name(), [this]() -> const bool & { return unity_online_; }, [this]() { unity_online_ = !unity_online_; }));
}

EXPORT_SINGLE_STATE("OrientationRetargetting", OrientationRetargetting)
