#include "Initial_ISMPC.h"

#include <ismpc_walking/Walking_controller.h>

namespace avatar
{

void Initial_ISMPC::start(mc_control::fsm::Controller & ctl)
{
  if(auto target = config_.find("target")) { ctl.getPostureTask(ctl.robot().name())->target(*target); }
  ctl.gui()->addElement(this, {},
                        mc_rtc::gui::Button("Start avatar control",
                                            [this, &ctl]()
                                            {
                                              auto & walking = dynamic_cast<Walking_controller &>(ctl);
                                              // FIXME Make activate in ISMPC public?
                                              walking.gui()->handleRequest({"Walking", "Main"}, "Active", {});
                                              output("Standing");
                                            }));
}

bool Initial_ISMPC::run(mc_control::fsm::Controller &)
{
  return !output().empty();
}

void Initial_ISMPC::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

} // namespace avatar

EXPORT_SINGLE_STATE("Avatar::Initial::ISMPC", avatar::Initial_ISMPC)
