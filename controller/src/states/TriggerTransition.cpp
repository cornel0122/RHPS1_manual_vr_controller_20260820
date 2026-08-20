#include "TriggerTransition.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"

void TriggerTransition::configure(const mc_rtc::Configuration & config)
{
  config("datastoreJoy", datastoreJoy_);
  // mc_rtc::log::info("[{}] AXES MAP: {}",name(), config("axesTrigger").dump(true));
  config("axesTrigger", axesTrigger_);
}

void TriggerTransition::start(mc_control::fsm::Controller & ctl)
{
  if(!ctl.datastore().has(datastoreJoy_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No joystick {} on the datastore", name(), datastoreJoy_);
  }
}

bool TriggerTransition::run(mc_control::fsm::Controller & ctl)
{
  const auto & joy = ctl.datastore().get<SubscriberData<OcculusHandJoystick>>(datastoreJoy_).value();

  for(const auto & trigger : axesTrigger_)
  {
    const auto & triggerAxis = trigger.first;
    for(const auto & triggerMap : trigger.second)
    {
      if(triggerAxis == "PrimaryTrigger" && std::abs(joy.primary_trigger - triggerMap.first) < 0.001)
      {
        mc_rtc::log::success("[{}] Triggering transition to {}", name(), triggerMap.second);
        output(triggerMap.second);
        return true;
      }
      else if(triggerAxis == "SecondaryTrigger" && std::abs(joy.secondary_trigger - triggerMap.first) < 0.001)
      {
        mc_rtc::log::success("[{}] Triggering transition to {}", name(), triggerMap.second);
        output(triggerMap.second);
        return true;
      }
    }
  }
  return false;
}

void TriggerTransition::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("TriggerTransition", TriggerTransition)
