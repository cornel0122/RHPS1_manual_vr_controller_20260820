#include "JoystickControl.h"

#include "../ANAAvatarController.h"

void JoystickControl::configure(const mc_rtc::Configuration & config)
{
  if(auto native_cfg = config.find("Joystick"))
  {
    (*native_cfg)("left_retargeting_state_name", native_joystick_.left_hand_retargeting_name);
    (*native_cfg)("right_retargeting_state_name", native_joystick_.right_hand_retargeting_name);
  }
  if(auto ros_cfg = config.find("ROS"))
  {
    (*ros_cfg)("walking_dir_ros_topic", ros_joystick_.walking_dir_topic_);
    (*ros_cfg)("forward_trigger_ros_topic", ros_joystick_.forward_trigger_topic_);
    (*ros_cfg)("stride_trigger_ros_topic", ros_joystick_.stride_trigger_topic_);
    if(auto ros_joy_cfg = ros_cfg->find("Joy"))
    {
      (*ros_joy_cfg)("timeout", ros_joy_msg_joystick_.timeout_);
      (*ros_joy_cfg)("deadzone", ros_joy_msg_joystick_.deadzone_);
      (*ros_joy_cfg)("topic", ros_joy_msg_joystick_.topic_);
      (*ros_joy_cfg)("forward_axis", ros_joy_msg_joystick_.forward_axis_);
      (*ros_joy_cfg)("side_axis", ros_joy_msg_joystick_.side_axis_);
      (*ros_joy_cfg)("rot_axis", ros_joy_msg_joystick_.rot_axis_);
    }
  }
  if(auto unity_head_cfg = config.find("Unity", "HeadJoystick"))
  {
    (*unity_head_cfg)("datastore_reference", unity_head_joystick_.datastoreRetargetingReference_);
    (*unity_head_cfg)("max_rot_yaw", unity_head_joystick_.max_rot_yaw_);
    (*unity_head_cfg)("min_rot_yaw", unity_head_joystick_.min_rot_yaw_);
    (*unity_head_cfg)("max_vel_pitch", unity_head_joystick_.max_vel_pitch_);
    (*unity_head_cfg)("min_vel_pitch", unity_head_joystick_.min_vel_pitch_);
    (*unity_head_cfg)("orientation_offset", unity_head_joystick_.head_reference_frame_offset);
  }
  if(auto unity_cfg = config.find("Unity", "Joystick"))
  {
    (*unity_cfg)("timeout", unity_joystick_.timeout_);
    (*unity_cfg)("deadzone", unity_joystick_.deadzone_);
  }
  config("head_retargeting_state_name", settings.head_retargeting_name);
  config("max_vel_forward", settings.max_vel_forward);
  config("max_vel_backward", settings.max_vel_backward);
  config("max_vel_side", settings.max_vel_side);
  config("max_vel_rot", settings.max_vel_rot);
  config("retargeting_state_names", retargeting_state_names);
  if(auto modality = config.find<std::string>("modality")) { modality_ = from_string(*modality); }
}

void JoystickControl::start(mc_control::fsm::Controller & ctl)
{
  settings.name = name();
  settings.available_modalities = {"ManualSpeed", "UnityHead", "UnityJoystick"};
  native_joystick_.start(ctl, settings);
  ros_joystick_.start(ctl, settings);
  ros_joy_msg_joystick_.start(ctl, settings);

  ctl.gui()->addElement({}, mc_rtc::gui::Checkbox(
                                "Walking mode", [this]() { return walking_mode_enabled_; },
                                [this, &ctl]() { toggle_walking_mode(ctl); }));

  // Modality selector
  ctl.gui()->addElement({"Avatar", name()},
                        mc_rtc::gui::ComboInput(
                            "Modality", settings.available_modalities, [this]() { return to_string(modality_); },
                            [this](const std::string & mod) { modality_ = from_string(mod); }));

  ctl.gui()->addElement({"Avatar", name()}, mc_rtc::gui::NumberInput("Max forward velocity", settings.max_vel_forward),
                        mc_rtc::gui::NumberInput("Max backward velocity", settings.max_vel_backward),
                        mc_rtc::gui::NumberInput("Max side velocity", settings.max_vel_side),
                        mc_rtc::gui::NumberInput("Max rotation velocity (rad/s)", settings.max_vel_rot));

  unity_head_joystick_.start(ctl, settings);
  unity_joystick_.start(ctl, settings);
  manual_speed_.start(ctl, settings);

  if(std::find(settings.available_modalities.begin(), settings.available_modalities.end(), to_string(modality_))
     == settings.available_modalities.end())
  {
    mc_rtc::log::critical("Required joystick modality ({}) is not available, disabling joystick", to_string(modality_));
    modality_ = Modality::None;
  }
}

bool JoystickControl::run(mc_control::fsm::Controller & ctl)
{
  switch(modality_)
  {
    case Modality::ManualSpeed:
      manual_speed_.run(ctl, settings);
      break;
    case Modality::Native:
      native_joystick_.run(ctl, settings);
      break;
    case Modality::ROS:
      ros_joystick_.run(ctl, settings);
      break;
    case Modality::ROSJoy:
      ros_joy_msg_joystick_.run(ctl, settings);
      break;
    case Modality::UnityHead:
      unity_head_joystick_.run(ctl, settings);
      break;
    case Modality::UnityJoystick:
      unity_joystick_.run(ctl, settings);
      break;
    default:
      break;
  }
  return true;
}
void JoystickControl::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  // FIXME Implement teardown in modalities
}

void JoystickControl::toggle_walking_mode(mc_control::fsm::Controller & ctl)
{
  auto & ds = ctl.datastore();
  walking_mode_enabled_ = !walking_mode_enabled_;
  for(const auto & s : retargeting_state_names)
  {
    std::string toggle = s + "::toggle_walking_mode";
    ds.call(toggle);
  }
}

EXPORT_SINGLE_STATE("JoystickControl", JoystickControl)
