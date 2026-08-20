#include "CoMRetargetting.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/io_utils.h>
#include "../ANAAvatarController.h"
#include "../ROSSubscriber.h"

void CoMRetargetting::configure(const mc_rtc::Configuration & config)
{
  config("datastorePoseWorld", datastorePoseWorld_); // output pose
  config("datastoreJoy", datastoreJoy_);
  config("scaling", scaling_);
  config("minCoMHeight", minCoMHeight_);
  config("minPressTime", minPressTime_);
  config("moveCoMDuration", moveCoMDuration_);
  if(config.has("minTorsoPitch")) { minTorsoPitch_ = mc_rtc::constants::toRad(config("minTorsoPitch")); }
  if(config.has("maxTorsoPitch")) { maxTorsoPitch_ = mc_rtc::constants::toRad(config("maxTorsoPitch")); }
  config("originOffset", originOffset_);

  config("state_ros_topic", state_topic_);
  config("trigger_ros_topic", trigger_topic_);
  config_.load(config);
}

void CoMRetargetting::start(mc_control::fsm::Controller & ctl)
{
  // if(!ctl.datastore().has(datastoreJoy_))
  //{
  //  mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick {} on the datastore", name(), datastoreJoy_);
  //}
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");

  chestInitHeight_ = walking.get_com_height();
  defaultCoMHeight_ = walking.get_com_height();

  ctl.datastore().make<bool>("ANA::ComTracking", false);
  ctl.datastore().make_call("ANA::ComTracking::Trigger", [this]() { active_trigger_ = true; });
  ctl.datastore().make<bool>("ANA::ComTracking::CoMUp", true);

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    state_trigger_sub_.subscribe(*nh_, trigger_topic_);
    state_trigger_sub_.maxTime(maxTime_);
    height_state_pub_ = nh_->advertise<std_msgs::Bool>(state_topic_, 1);
    state_trigger_pub_ = nh_->advertise<std_msgs::Bool>(trigger_topic_, 1);
  }

  ctl.gui()->addElement(this, {"Avatar", name()}, mc_rtc::gui::Label("Active", [this]() { return active_; }));
  ctl.gui()->addElement(
      this, {"Avatar", name()}, mc_rtc::gui::Button("Trigger Up/Down", [this]() { active_trigger_ = true; }),
      mc_rtc::gui::NumberInput(
          "Min Torso Pitch", [this]() { return mc_rtc::constants::toDeg(minTorsoPitch_); },
          [this](double pitch) { minTorsoPitch_ = mc_rtc::constants::toRad(pitch); }),

      mc_rtc::gui::NumberInput(
          "Max Torso Pitch", [this]() { return mc_rtc::constants::toDeg(maxTorsoPitch_); },
          [this](double pitch) { maxTorsoPitch_ = mc_rtc::constants::toRad(pitch); }),
      mc_rtc::gui::NumberInput(
          "Min CoM height [m]", [this]() { return minCoMHeight_; }, [this](double height) { minCoMHeight_ = height; }),
      mc_rtc::gui::NumberInput(
          "Scaling [0;1]", [this]() { return scaling_; },
          [this](double scale) { scaling_ = mc_filter::utils::clamp(scale, 0, 1); }));
  ctl.gui()->addElement(this, {"Avatar", name(), "Offsets"},

                        mc_rtc::gui::ArrayInput(
                            "Origin offset (translation) [m]", {"x", "y", "z"},
                            [this]() -> const Eigen::Vector3d & { return originOffset_.translation(); },
                            [this](const Eigen::Vector3d & t) { originOffset_.translation() = t; }),
                        mc_rtc::gui::ArrayInput(
                            "Origin offset (rotation) [deg]", {"r", "p", "y"},
                            [this]() -> Eigen::Vector3d
                            { return mc_rbdyn::rpyFromMat(originOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                            [this](const Eigen::Vector3d & rpy)
                            { originOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));

  ctl.gui()->addElement(this, {"Avatar", "Unity", "State"},
                        mc_rtc::gui::Checkbox(
                            "CoMDown", [this]() { return !com_up_ || active_; }, [this]() {}));

  ctl.gui()->addElement(
      this, {"Avatar", "Unity", "Trigger"},
      mc_rtc::gui::Checkbox(
          name(), [this]() { return active_trigger_; }, [this]() { active_trigger_ = !active_trigger_; }));

  auto & logger = ctl.logger();
  MC_RTC_LOG_HELPER(name() + "_retargetting_active", active_);
  MC_RTC_LOG_HELPER(name() + "_move_com_active", moveCoMActive_);
}

bool CoMRetargetting::run(mc_control::fsm::Controller & ctl)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  comHeight_ = ctl.robot().com().z();
  ctl.datastore().assign("ANA::ComTracking", false);
  auto robot_walking_func = [&]() { return walking.is_walking(); };
  auto com_height_func = [&](double h) { walking.set_com_height(h); };
  auto torso_pitch_func = [&](double p) { walking.set_torso_pitch(p); };
  // bool pauseWalking = ctl.datastore().call<bool>("Walking::IsPaused");
  bool pauseWalking = !robot_walking_func();

  if(ctl.datastore().get<bool>("UseROS"))
  {
    state_trigger_sub_.tick(ctl.solver().dt());
    if(state_trigger_sub_.data().value() && trigger_on_count_ * ctl.timeStep > 1)
    {
      active_trigger_ = true;
      mc_rtc::log::info("[{}] trigger received", name());
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      trigger_on_count_ = 0;
      state_trigger_pub_.publish(trigger_msg);
    }
    trigger_on_count_++;
  }

  if(active_trigger_)
  {
    active_trigger_ = false;
    active_ = pauseWalking || ctl.datastore().get<bool>("Emergency");
    if(!pauseWalking)
    {
      mc_rtc::log::warning("[{}] CoM retargetting failed to activate because walking is not paused");
    }
    moveUp_ = !moveUp_;
    if(moveUp_) { mc_rtc::log::info("[{}] Moving CoM Up", name()); }
    else { mc_rtc::log::info("[{}] Moving CoM Down", name()); }
    moveCoMIter_ = 0;
    active_ = true;
    moveCoMInitHeight_ = ctl.robot().com().z();
  }

  if(ctl.datastore().get<bool>("Emergency")) { active_ = false; }

  if(active_)
  {
    auto ratio = (moveCoMIter_ * ctl.timeStep) / moveCoMDuration_;
    if(moveUp_) { comHeight_ = moveCoMInitHeight_ + ratio * (defaultCoMHeight_ - moveCoMInitHeight_); }
    else { comHeight_ = moveCoMInitHeight_ - ratio * (moveCoMInitHeight_ - minCoMHeight_); }
    if(ratio >= 1)
    {
      active_ = false;
      moveCoMIter_ = 0;
      if(moveUp_) { com_up_ = true; }
      else { com_up_ = false; }
      ctl.datastore().assign("ANA::ComTracking", com_up_);
    }
    ++moveCoMIter_;

    mc_filter::utils::clampInPlace(comHeight_, minCoMHeight_, defaultCoMHeight_);
    com_height_func(comHeight_);

    ratio = (defaultCoMHeight_ - comHeight_) / (defaultCoMHeight_ - minCoMHeight_);
    torsoPitch_ = minTorsoPitch_ + ratio * (maxTorsoPitch_ - minTorsoPitch_);
    torso_pitch_func(torsoPitch_);

    ctl.datastore().assign("ANA::ComTracking", true);
  }
  else
  {
    ctl.datastore().assign("ANA::ComTracking", false);
    walkingRequested_ = false;
  }

  ctl.datastore().assign("ANA::ComTracking::CoMUp", com_up_);

  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Bool state_msg;
    state_msg.data = !com_up_ || active_;
    height_state_pub_.publish(state_msg);
  }

  output("OK");
  return true;
}

void CoMRetargetting::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
}

EXPORT_SINGLE_STATE("CoMRetargetting", CoMRetargetting)
