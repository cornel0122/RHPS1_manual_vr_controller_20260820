#include "GripperHandling.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"

void GripperHandling::configure(const mc_rtc::Configuration & config)
{
  config("datastoreJoy", datastoreJoy_);
  config("gripper_opening_topic", gripper_opening_topic_);
  config("thumbPercentOpen", thumbPercentOpen_);
  config("thumbPercentClosed", thumbPercentClosed_);
  config("fingersPercentOpen", fingersPercentOpen_);
  config("fingersPercentClosed", fingersPercentClosed_);
  config("safety", safety_);
  config_.load(config);
}

void GripperHandling::start(mc_control::fsm::Controller & ctl)
{
  if(!config_.has("robot"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No {} defined on the datastore", name(), "robot");
  }
  if(!config_("robot").has(ctl.robot().name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No {} defined on the datastore", name(), ctl.robot().name());
  }

  config_("robot")(ctl.robot().name())("gripper", gripper_);
  config_("robot")(ctl.robot().name())("thumbJoint", thumbJoint_);
  config_("robot")(ctl.robot().name())("fingersJoint", fingersJoint_);

  // if(!ctl.datastore().has(datastoreJoy_))
  //{
  //  mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick {} on the datastore", name(), datastoreJoy_);
  //}
  if(!ctl.robot().hasGripper(gripper_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No gripper \"{}\" in robot \"{}\"", name(), gripper_,
                                                     ctl.robot().name());
  }
  auto & gripper = ctl.robot().gripper(gripper_);
  if(!gripper.hasActiveJoint(thumbJoint_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] Gripper \"{}\" does not have an active thumb joint named \"{}\"", name(), gripper_, thumbJoint_);
  }
  if(!gripper.hasActiveJoint(fingersJoint_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] Gripper \"{}\" does not have an active fingers joint named \"{}\"", name(), gripper_, fingersJoint_);
  }

  if(safety_.has("threshold")) { gripper.actualCommandDiffTrigger(mc_rtc::constants::toRad(safety_("threshold"))); }
  if(safety_.has("iter")) { gripper.overCommandLimitIterN(safety_("iter")); }
  if(safety_.has("release")) { gripper.releaseSafetyOffset(mc_rtc::constants::toRad(safety_("release"))); }
  if(safety_.has("vmaxPercent")) { gripper.percentVMAX(safety_("vmaxPercent")); }

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    gripper_openning_sub_.subscribe(*nh_, gripper_opening_topic_);
    gripper_openning_sub_.maxTime(maxTime_);
  }

  ctl.gui()->addElement(
      this, {"Avatar", name()},
      mc_rtc::gui::Button("Toggle thumb opening", [this, &ctl]() { openCloseThumb(ctl.robot().gripper(gripper_)); }),
      mc_rtc::gui::NumberInput(
          "Thumb percent closed", [this]() { return thumbPercentClosed_; },
          [this](double v) { thumbPercentClosed_ = v; }),
      mc_rtc::gui::NumberInput(
          "Thumb percent open", [this]() { return thumbPercentOpen_; }, [this](double v) { thumbPercentOpen_ = v; }),
      mc_rtc::gui::Button("Toggle fingers opening",
                          [this, &ctl]() { openCloseFingers(ctl.robot().gripper(gripper_)); }),
      mc_rtc::gui::NumberInput(
          "Fingers percent closed", [this]() { return fingersPercentClosed_; },
          [this](double v) { fingersPercentClosed_ = v; }),
      mc_rtc::gui::NumberInput(
          "Fingers percent open", [this]() { return fingersPercentOpen_; },
          [this](double v) { fingersPercentOpen_ = v; }));

  ctl.gui()->addElement(this, {"Avatar", "Unity", "Grippers"},
                        mc_rtc::gui::ArrayInput(
                            name(), [this]() { return std::vector<double>{display_target_opening_}; },
                            [this](std::vector<double> t)
                            {
                              display_target_opening_ = t[0];
                              target_opening_ = t[0] / 2;
                            }));
  ctl.gui()->addElement(this, {"Avatar", "Unity", "Grippers"},
                        mc_rtc::gui::ArrayInput(
                            name() + "RobotOpening",
                            [this, &ctl]() { return std::vector<double>{ctl.robot().gripper(gripper_).curOpening()}; },
                            [this](std::vector<double> t) {}));
}

void GripperHandling::openCloseThumb(mc_control::Gripper & gripper)
{
  if(closingThumb)
  {
    mc_rtc::log::info("[{}] Closing gripper thumb", name());
    gripper.setTargetOpening(thumbJoint_, thumbPercentClosed_);
    closingThumb = false;
  }
  else
  {
    mc_rtc::log::info("[{}] Opening gripper thumb", name());
    gripper.setTargetOpening(thumbJoint_, thumbPercentOpen_);
    closingThumb = true;
  }
}

void GripperHandling::openCloseFingers(mc_control::Gripper & gripper)
{
  if(closingFingers)
  {
    mc_rtc::log::info("[{}] Closing gripper fingers", name());
    gripper.setTargetOpening(fingersJoint_, fingersPercentClosed_);
    closingFingers = false;
  }
  else
  {
    mc_rtc::log::info("[{}] Opening gripper fingers", name());
    gripper.setTargetOpening(fingersJoint_, fingersPercentOpen_);
    closingFingers = true;
  }
}

bool GripperHandling::run(mc_control::fsm::Controller & ctl)
{
  // const auto & joy = ctl.datastore().get<SubscriberData<OcculusHandJoystick>>(datastoreJoy_).value();
  auto & gripper = ctl.robot().gripper(gripper_);

  // if(joy.xClicked)
  //{
  //  mc_rtc::log::success("X clicked");
  //  openCloseThumb(gripper);
  //}

  // if(joy.yClicked)
  //{
  //  openCloseFingers(gripper);
  //}
  double target_opening = target_opening_;
  if(ctl.datastore().get<bool>("UseROS") && true)
  {
    target_opening = mc_filter::utils::clamp(gripper_openning_sub_.data().value(), 0, 1);

    // gripper.setTargetOpening(thumbJoint_, gripper_state.value());
  }
  gripper.setTargetOpening(fingersJoint_, target_opening);

  output("OK");
  return true;
}

void GripperHandling::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("GripperHandling", GripperHandling)
