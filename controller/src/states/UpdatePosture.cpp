#include "UpdatePosture.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"

void UpdatePosture::start(mc_control::fsm::Controller & ctl)
{
  config_("update_posture", update_posture_);
  run(ctl);
}

bool UpdatePosture::run(mc_control::fsm::Controller & ctl)
{
  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(update_posture_) { posture->reset(); }
  else
  {
    // Set half-sitting pose for posture task
    auto pos = posture->posture();
    const auto & halfSit = ctl.robot().module().stance();
    const auto & refJointOrder = ctl.robot().refJointOrder();
    for(unsigned i = 0; i < refJointOrder.size(); ++i)
    {
      auto idx = ctl.robot().jointIndexInMBC(i);
      if(idx < 0) { continue; }
      auto mbcIdx = static_cast<size_t>(idx);
      if(pos[mbcIdx].size() == 1) { pos[mbcIdx][0] = halfSit.at(refJointOrder[i])[0]; }
    }
  }

  output("OK");
  return true;
}

void UpdatePosture::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("UpdatePosture", UpdatePosture)
