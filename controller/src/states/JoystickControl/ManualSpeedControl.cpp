#include "common.h"

void ManualSpeedControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  ctl.gui()->addElement({"Avatar", settings.name, "Manual::Speed"},
                        mc_rtc::gui::ArrayInput(
                            "Speed", {"Forward", "Move left", "Turn left [deg/s]"},
                            [this]() -> const Eigen::Vector3d & { return ref_vel_; },
                            [this, &settings](const Eigen::Vector3d & velIn)
                            {
                              ref_vel_.x() = std::clamp(velIn.x(), settings.max_vel_backward, settings.max_vel_forward);
                              ref_vel_.y() = std::clamp(velIn.y(), -settings.max_vel_side, settings.max_vel_side);

                              ref_vel_.z() =
                                  std::clamp(velIn.z() * M_PI / 180, -settings.max_vel_rot, settings.max_vel_rot);
                            }));
}

void ManualSpeedControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings &)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  if(ref_vel_.norm() > 1e-6 && !walking.is_walking()) { walking.start_stop_walking(); }
  if(ref_vel_.norm() < 1e-6 && !walking.is_stopping()) { walking.start_stop_walking(); }
  walking.set_planner_ref_vel(ref_vel_);
}
