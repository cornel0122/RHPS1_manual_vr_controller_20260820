#pragma once

#include <mc_control/fsm/State.h>

namespace avatar
{

struct Initial_ISMPC : public mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller &) override;
  void teardown(mc_control::fsm::Controller & ctl) override;
};

} // namespace avatar
