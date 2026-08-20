#include <mc_control/GlobalPlugin.h>

namespace mc_control
{

struct LoadForceConstrainedTransformTask : public GlobalPlugin
{
  GlobalPluginConfiguration configuration() override;

  void init(MCGlobalController &, const Configuration &) override {}
  void reset(MCGlobalController &) override {}
  void before(MCGlobalController &) override {}
  void after(MCGlobalController &) override {}

private:
  void build(MCGlobalController &);
};

} // namespace mc_control
