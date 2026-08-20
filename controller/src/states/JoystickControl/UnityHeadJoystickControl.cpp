#include "common.h"

void UnityHeadJoystickControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  const auto & name = settings.name;
  ctl.gui()->addElement({"Avatar", name, "Unity::HeadJoystick"},
                        mc_rtc::gui::ArrayInput(
                            "Head ori [deg]", {"r", "p", "y"},
                            [this]() -> Eigen::Vector3d { return head_rpy * 180. / mc_rtc::constants::PI; },
                            [](const Eigen::Vector3d &) {}),
                        mc_rtc::gui::ArrayInput(
                            "Head ori threshold [deg]", {"min_yaw", "max_yaw", "min_pitch", "max_pitch"},
                            [this]() -> std::array<double, 4> {
                              return {min_rot_yaw_, max_rot_yaw_, min_vel_pitch_, max_vel_pitch_};
                            },
                            [this](const std::array<double, 4> & in)
                            {
                              min_rot_yaw_ = in[0];
                              max_rot_yaw_ = in[1];
                              min_vel_pitch_ = in[2];
                              max_vel_pitch_ = in[3];
                            }));
  ctl.gui()->addElement(
      {"Avatar", "Unity", "Trigger"},
      mc_rtc::gui::Checkbox(
          name + "Stand", []() { return false; }, [this]() { operator_walking_direction = WalkingDirection::Stand; }),
      mc_rtc::gui::Checkbox(
          name + "Forward", []() { return false; },
          [this]() { operator_walking_direction = WalkingDirection::Forward; }),
      mc_rtc::gui::Checkbox(
          name + "Side", []() { return false; }, [this]() { operator_walking_direction = WalkingDirection::Side; }),
      mc_rtc::gui::Checkbox(
          name + "Backward", []() { return false; },
          [this]() { operator_walking_direction = WalkingDirection::Backward; }));
}

void UnityHeadJoystickControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");

  auto & synchro_active_func = ctl.datastore().get<std::function<bool(void)>>("synchro_walk::active");
  if(!synchro_active_func()) { operator_walking_direction = WalkingDirection::Stand; };

  auto & head_task =
      ctl.datastore().get<std::shared_ptr<mc_tasks::TransformTask>>(settings.head_retargeting_name + "Task");
  const auto X_0_chest_reference = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReference_);
  const auto X_0_head = head_task->frame().position();
  sva::PTransformd X_reference_head = head_reference_frame_offset * X_0_head * X_0_chest_reference.inv();
  head_rpy = mc_rbdyn::rpyFromMat(X_reference_head.rotation());

  ref_vel_ = Eigen::Vector3d::Zero();
  if(operator_walking_direction == WalkingDirection::Stand) { ref_vel_.setZero(); }

  else if(operator_walking_direction == WalkingDirection::Forward
          || operator_walking_direction == WalkingDirection::TurnInPlace)
  {
    if((head_rpy.z() < min_rot_yaw_ * 3.14 / 180 && head_rpy.z() > -min_rot_yaw_ * 3.14 / 180))
    {
      if(operator_walking_direction == WalkingDirection::TurnInPlace)
      {
        operator_walking_direction = WalkingDirection::Forward;
      }
      double vel_ratio_forward = (max_vel_pitch_ - head_rpy.y() * (180 / 3.14)) / (max_vel_pitch_ - min_vel_pitch_);
      mc_filter::utils::clampInPlace(vel_ratio_forward, 0, 1);
      ref_vel_.x() = vel_ratio_forward * settings.max_vel_forward;
    }

    else if((std::abs(head_rpy.z()) > min_rot_yaw_ * 3.14 / 180 && std::abs(head_rpy.z()) < max_rot_yaw_ * 3.14 / 180)
            && operator_walking_direction == WalkingDirection::Forward)
    {

      double sgn = head_rpy.z() / std::abs(head_rpy.z());
      double vel_ratio_forward = (max_vel_pitch_ - head_rpy.y() * (180 / 3.14)) / (max_vel_pitch_ - min_vel_pitch_);
      mc_filter::utils::clampInPlace(vel_ratio_forward, 0, 1);
      double vel_ratio_rot =
          (sgn * min_rot_yaw_ - head_rpy.z() * (180 / 3.14)) / (sgn * min_rot_yaw_ - sgn * max_rot_yaw_);
      mc_filter::utils::clampInPlace(vel_ratio_rot, 0, 1);

      ref_vel_.z() = vel_ratio_rot * settings.max_vel_rot * sgn;
      ref_vel_.x() = vel_ratio_forward * settings.max_vel_forward;
    }

    else if(std::abs(head_rpy.z()) > max_rot_yaw_ * 3.14 / 180
            || operator_walking_direction == WalkingDirection::TurnInPlace)
    {
      operator_walking_direction = WalkingDirection::TurnInPlace;
      ref_vel_.x() = 0.00;
      double sgn = head_rpy.z() / std::abs(head_rpy.z());
      ref_vel_.z() = sgn * settings.max_vel_rot;
    }
  }

  else if(operator_walking_direction == WalkingDirection::Backward) { ref_vel_.x() = settings.max_vel_backward; }

  else if(operator_walking_direction == WalkingDirection::Side)
  {
    if(std::abs(head_rpy.z()) > min_rot_yaw_ * 3.14 / 180)
    {

      double sgn = head_rpy.z() / std::abs(head_rpy.z());
      ref_vel_.y() = sgn * settings.max_vel_side;
    }
  }

  if(walking.is_double_support()) { walking.set_planner_ref_vel(ref_vel_); }
}
