#include "plugin.h"

#include <mc_control/GlobalPluginMacros.h>

#include <mc_tasks/ForceConstrainedTransformTask.h>

namespace mc_control
{

auto LoadForceConstrainedTransformTask::configuration() -> GlobalPluginConfiguration
{
  GlobalPluginConfiguration out;
  out.should_always_run = false;
  out.should_run_before = false;
  out.should_run_after = false;
  return out;
}

// Brings-in the task's symbols
void LoadForceConstrainedTransformTask::build(MCGlobalController & controller)
{
  mc_tasks::ForceConstrainedTransformTask task(controller.robot().frame("LeftGripper"));
}

} // namespace mc_control

EXPORT_MC_RTC_PLUGIN("LoadForceConstrainedTransformTask", mc_control::LoadForceConstrainedTransformTask)

extern "C"
{
  GLOBAL_PLUGIN_API void LOAD_GLOBAL() {}
}
