#include "JointTrackingPlotterPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

using namespace mc_rtc::gui;

JointTrackingPlotter::~JointTrackingPlotter()
{
  mc_rtc::log::info("JointTrackingPlotter destroyed");
}

void JointTrackingPlotter::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("JointTrackingPlotter initialized with configuration:\n{}", config.dump(true, true));
  auto & ctl = controller.controller();

  auto plotJoint = [this, &ctl](const std::string & joint)
  {
    mc_rtc::log::info("[JointTrackingPlotter] Plotting joint: {}", joint);
    if(std::find(joints_.begin(), joints_.end(), joint) != joints_.end()) { return; }

    const auto & rjo = ctl.robot().refJointOrder();
    auto jit = std::find(rjo.begin(), rjo.end(), joint);
    if(jit == rjo.end())
    {
      mc_rtc::log::error("[JointTrackingPlotter] No encoder for joint \"{}\"", joint);
      return;
    }

    auto jidx = std::distance(rjo.begin(), jit);

    if(ctl.robot().mbc().q[ctl.robot().jointIndexByName(joint)].size() != 1)
    {
      mc_rtc::log::error("[JointTrackingPlotter] This plotter only supports 1dof joints");
      return;
    }
    ctl.gui()->addPlot(joint + "_position", plot::X("t", [this]() { return t_; }),
                       plot::Y(
                           "error command-encoder (deg)",
                           [&ctl, joint, jidx]() -> double
                           {
                             return mc_rtc::constants::toDeg(ctl.robot().mbc().q[ctl.robot().jointIndexByName(joint)][0]
                                                             - ctl.robot().encoderValues()[jidx]);
                           },
                           Color::Blue));
    if(ctl.robot().jointTorques().size() == ctl.robot().refJointOrder().size())
    {
      ctl.gui()->addPlot(joint + "_torque", plot::X("t", [this]() { return t_; }),
                         plot::Y(
                             "measured torque",
                             [&ctl, joint, jidx]() -> double { return ctl.robot().jointTorques()[jidx]; },
                             Color::Blue));
    }
    else { mc_rtc::log::warning("[JointTrackingPlotter] No torque measurement for joint {}", joint); }
    joints_.push_back(joint);
  };

  ctl.gui()->addElement(this, category_,
                        mc_rtc::gui::DataComboInput(
                            "Joints", std::vector<std::string>{"joints", ctl.robot().name()},
                            [this]() { return joint_; }, [this](const std::string & joint) { joint_ = joint; }),
                        mc_rtc::gui::Button("Add plot", [plotJoint, this]() { plotJoint(joint_); }),
                        mc_rtc::gui::Button("Remove plot",
                                            [this, &ctl]()
                                            {
                                              auto it = std::find(joints_.begin(), joints_.end(), joint_);
                                              if(it != joints_.end())
                                              {
                                                ctl.gui()->removePlot(joint_ + "_position");
                                                ctl.gui()->removePlot(joint_ + "_torque");
                                                joints_.erase(it);
                                              }
                                            }));
}

void JointTrackingPlotter::reset(mc_control::MCGlobalController &) {}

void JointTrackingPlotter::before(mc_control::MCGlobalController &) {}

void JointTrackingPlotter::after(mc_control::MCGlobalController & controller)
{
  t_ += controller.controller().timeStep;
}

mc_control::GlobalPlugin::GlobalPluginConfiguration JointTrackingPlotter::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = false;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("JointTrackingPlotterPlugin", mc_plugin::JointTrackingPlotter)
