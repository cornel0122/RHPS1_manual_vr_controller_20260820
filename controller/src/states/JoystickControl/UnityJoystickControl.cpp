#include "common.h"

void UnityJoystickControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  ctl.gui()->addElement(
      {"Avatar", settings.name, "Unity::Joystick"}, mc_rtc::gui::ArrayInput("Deadzone", deadzone_),
      mc_rtc::gui::ArrayLabel("Raw inputs", [this]() -> const std::array<double, 3> & { return data_; }),
      mc_rtc::gui::ArrayLabel("Filtered inputs",
                              [this]() -> std::array<double, 3> {
                                return {forward_input_, side_input_, rot_input_};
                              }));
  ctl.gui()->addElement(
      {"Avatar", "Unity", "Joystick"},
      mc_rtc::gui::ArrayInput(
          "Stick", {"Forward", "Move left", "Turn left"}, [this]() -> const std::array<double, 3> & { return data_; },
          [this](const std::array<double, 3> & data)
          {
            data_ = data;
            forward_input_ = deadzoned_input(data[0], deadzone_.x());
            side_input_ = deadzoned_input(data[1], deadzone_.y());
            rot_input_ = deadzoned_input(data[2], deadzone_.z());
            last_update_ = 0.0;
          }),
      mc_rtc::gui::ArrayInput(
          "Velocity", [this]() -> const Eigen::Vector3d & { return ref_vel_; }, [](const std::array<double, 3> &) {}));
}

void UnityJoystickControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  last_update_ += ctl.solver().dt();
  if(last_update_ > timeout_ && !walking.is_stopping())
  {
    mc_rtc::log::critical("[{}::UnityJoystick] No data received for {}s, stop walking now", settings.name, timeout_);
    walking.set_planner_ref_vel(Eigen::Vector3d::Zero());
    walking.start_stop_walking();
    return;
  }
  if(forward_input_ > 0) { ref_vel_.x() = settings.max_vel_forward * forward_input_; }
  else { ref_vel_.x() = settings.max_vel_backward * abs(forward_input_); }
  ref_vel_.y() = settings.max_vel_side * side_input_;
  if(std::abs(ref_vel_.y()) > 0.3 * settings.max_vel_side) { ref_vel_.x() = 0.0; }
  ref_vel_.z() = settings.max_vel_rot * rot_input_;
  if(ref_vel_.norm() > 1e-6 && !walking.is_walking()) { walking.start_stop_walking(); }
  if(ref_vel_.norm() < 1e-6 && !walking.is_stopping()) { walking.start_stop_walking(); }
  walking.set_planner_ref_vel(ref_vel_);
}
