#include "TriggerWalking.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"
#include <ismpc_walking/Walking_controller.h>

void TriggerWalking::configure(const mc_rtc::Configuration & config)
{
  config("datastoreLeftHandJoy", datastoreLeftHandJoy_);
  config("datastoreRightHandJoy", datastoreRightHandJoy_);
  config("triggerWalkingThreshold", triggerWalkingThreshold_);
}

void TriggerWalking::start(mc_control::fsm::Controller & ctl_)
{
  // auto & ctl = static_cast<lipm_walking::Controller &>(ctl_);
  // if(!ctl.datastore().has(datastoreLeftHandJoy_))
  // {
  //   mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick {} on the datastore", name(),
  //                                                    datastoreLeftHandJoy_);
  // }
  // if(!ctl.datastore().has(datastoreRightHandJoy_))
  // {
  //   mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick {} on the datastore", name(),
  //                                                    datastoreRightHandJoy_);
  // }
  // ctl.datastore().make_call("Walking::IsPaused", [&ctl]() { return ctl.pauseWalking; });
  // ctl.datastore().make_call("Walking::Pause", [&ctl](bool pause) { ctl.pauseWalking = pause; });
  // ctl.datastore().make_call("Walking::SetCoMHeight", [&ctl](double height) { ctl.plan.comHeight(height); });
  // ctl.datastore().make_call("Walking::GetCoMHeight", [&ctl]() { return ctl.plan.comHeight(); });
  // ctl.datastore().make_call("Walking::SetTorsoPitch", [&ctl](double pitch) { ctl.stabilizer()->torsoPitch(pitch); });
  // ctl.datastore().make_call("Walking::WalkingRequested", [this]() { return walkingRequested_; });
  mc_rtc::log::info("[{}] init", name());
}

bool TriggerWalking::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Walking_controller &>(ctl_);

  // const auto & ljoy = ctl.datastore().get<SubscriberData<OcculusHandJoystick>>(datastoreLeftHandJoy_).value();
  // const auto & rjoy = ctl.datastore().get<SubscriberData<OcculusHandJoystick>>(datastoreRightHandJoy_).value();

  // // Only allow walking if the CoM is close to the expected CoM
  // // CoMRetargetting state should ensure that this is the case
  // walkingRequested_ =
  //     !ctl.datastore().get<bool>("Emergency")
  //     && (std::abs(ljoy.vertical) > triggerWalkingThreshold_ || std::abs(ljoy.horizontal) > triggerWalkingThreshold_
  //         || std::abs(rjoy.horizontal) > triggerWalkingThreshold_);
  // bool isAllowedToWalk =
  //     std::fabs(ctl.robot().com().z() - ctl.robot().module().defaultLIPMStabilizerConfiguration().comHeight) <= 0.03;
  // bool startWalking = walkingRequested_ && isAllowedToWalk;
  // if(ctl.pauseWalking && startWalking)
  // {
  //   mc_rtc::log::success("[{}] Start walking", name());
  //   ctl.pauseWalking = false;
  // }
  // else if(!ctl.pauseWalking && !startWalking)
  // {
  //   mc_rtc::log::info("[{}] Stop walking", name());
  //   ctl.pauseWalking = true;
  // }
  // mc_rtc::log::info("[{}] run",name());

  output("OK");
  return true;
}

void TriggerWalking::teardown(mc_control::fsm::Controller & ctl)
{
  // ctl.datastore().remove("Walking::IsPaused");
  // ctl.datastore().remove("Walking::Pause");
  // ctl.datastore().remove("Walking::SetCoMHeight");
  // ctl.datastore().remove("Walking::GetCoMHeight");
}

EXPORT_SINGLE_STATE("TriggerWalking", TriggerWalking)
