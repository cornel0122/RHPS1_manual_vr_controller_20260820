#include "common.h"

void NativeJoystickControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  ctl.datastore().make<bool>("emergency_flag", false);
  if(!joystick.isFound())
  {
    connected = false;
    mc_rtc::log::warning("[{}] : NO JOYPAD DETECTED", settings.name);
    return;
  }
  settings.available_modalities.push_back(to_string(Modality::Native));
}

// Note: if this is called, the joystick is connected
void NativeJoystickControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  // Initialize callbacks if this wasn't done already
  if(!left_hand_on_func)
  {
    if(ctl.datastore().has(left_hand_retargeting_name + "::active"))
    {
      left_hand_on_func = ctl.datastore().get<std::function<bool(void)>>(left_hand_retargeting_name + "::active");
    }
  }
  if(!right_hand_on_func)
  {
    if(ctl.datastore().has(right_hand_retargeting_name + "::active"))
    {
      right_hand_on_func = ctl.datastore().get<std::function<bool(void)>>(right_hand_retargeting_name + "::active");
    }
  }

  // Take care of the inputs
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  while(joystick.sample(&event)) { handle_event(ctl, settings); }
  if(active) { walking.set_planner_ref_vel(input_vel_); }
}

void NativeJoystickControl::handle_event(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  if(event.isButton())
  {
    if(event.number == 2 && event.value == 1)
    {
      active = !active;
      mc_rtc::log::info("[{}::NativeJoystick] Walking input {}", settings.name, active);
    }
    if(event.number == 1 && event.value == 1)
    {
      mc_rtc::log::critical("[{}::NativeJoystick] SOFT EMERGENCY", settings.name);
      active = false;
      auto & stop_synchro_func = ctl.datastore().get<std::function<void(void)>>("synchro_walk::deactivate");
      stop_synchro_func();
      if(!walking.is_stopping()) { walking.start_stop_walking(); }
      if(ctl.datastore().has(left_hand_retargeting_name + "::active"))
      {
        if(left_hand_on_func())
        {
          auto on_off_func =
              ctl.datastore().get<std::function<void(void)>>(left_hand_retargeting_name + "::activate_deactivate");
          on_off_func();
        }
      }
      if(ctl.datastore().has(right_hand_retargeting_name + "::active"))
      {
        if(right_hand_on_func())
        {
          auto on_off_func =
              ctl.datastore().get<std::function<void(void)>>(right_hand_retargeting_name + "::activate_deactivate");
          on_off_func();
        }
      }
    }
    if(event.number == 0 && event.value == 1)
    {
      mc_rtc::log::info("[{}::NativeJoystick] Start/Stop", settings.name);
      walking.start_stop_walking();
    }

    if(event.number == 3 && event.value == 1)
    {
      mc_rtc::log::critical("[{}::NativeJoystick] EMERGENCY", settings.name);
      auto & val = ctl.datastore().get<bool>("emergency_flag");
      val = true;
    }
    if(event.value == 1 && event.number == 4)
    {
      if(ctl.datastore().has(left_hand_retargeting_name + "::activate_deactivate"))
      {
        auto on_off_func =
            ctl.datastore().get<std::function<void(void)>>(left_hand_retargeting_name + "::activate_deactivate");
        on_off_func();
      }
    }
    if(event.value == 1 && event.number == 5)
    {
      if(ctl.datastore().has(right_hand_retargeting_name + "::activate_deactivate"))
      {
        auto on_off_func =
            ctl.datastore().get<std::function<void(void)>>(right_hand_retargeting_name + "::activate_deactivate");
        on_off_func();
      }
    }
    if(event.value == 1 && event.number == 6)
    {
      if(ctl.datastore().has("synchro_walk::on_off"))
      {
        mc_rtc::log::info("[{}::NativeJoystick] Back button", settings.name);
        auto on_off_func = ctl.datastore().get<std::function<void(void)>>("synchro_walk::on_off");
        on_off_func();
      }
    }
    if(event.value == 1 && event.number == 7)
    {
      if(ctl.datastore().has(settings.head_retargeting_name + "::activate_deactivate"))
      {
        mc_rtc::log::info("[{}::NativeJoystick] Start button", settings.name);
        auto on_off_func =
            ctl.datastore().get<std::function<void(void)>>(settings.head_retargeting_name + "::activate_deactivate");
        on_off_func();
      }
    }
  }

  if(event.isAxis())
  {

    if(event.number == 1)
    {
      double value = -event.value;
      input_vel_.y() = -settings.max_vel_side + 2 * settings.max_vel_side * (value + 32767) / (32767 * 2);
      if(abs(input_vel_.y()) < 0.02) { input_vel_.y() = 0; }
    }
    if(event.number == 3)
    {
      double value = -event.value;
      input_vel_.z() = -settings.max_vel_rot + 2 * settings.max_vel_rot * (value + 32767) / (32767 * 2);
      if(abs(input_vel_.z()) < 0.03) { input_vel_.z() = 0; }
    }

    if(event.number == 2)
    {
      double value = -event.value;
      double maxVel = 0;
      double minVel = settings.max_vel_backward;
      input_vel_.x() = (maxVel - minVel) * (value + 32767) / (32767 * 2) + minVel;
      if(abs(input_vel_.x()) < 0.02) { input_vel_.x() = 0; }
    }

    if(event.number == 5)
    {
      double value = event.value;
      double maxVel = settings.max_vel_forward;
      double minVel = 0;
      input_vel_.x() = (maxVel - minVel) * (value + 32767) / (32767 * 2) + minVel;

      if(abs(input_vel_.x()) < 0.02) { input_vel_.x() = 0; }
    }
    if(event.number == 6)
    {
      if(static_cast<double>(event.value) == 32767)
      {
        if(ctl.datastore().has(right_hand_retargeting_name + "::activate_deactivate_prediction"))
        {
          auto on_off_func = ctl.datastore().get<std::function<void(void)>>(right_hand_retargeting_name
                                                                            + "::activate_deactivate_prediction");
          on_off_func();
        }
      }
      else if(static_cast<double>(event.value) == -32767)
      {
        // std::cout << "left pad" << std::endl;
        if(ctl.datastore().has(left_hand_retargeting_name + "::activate_deactivate_prediction"))
        {
          auto on_off_func = ctl.datastore().get<std::function<void(void)>>(left_hand_retargeting_name
                                                                            + "::activate_deactivate_prediction");
          on_off_func();
        }
      }
    }
    if(event.number == 7)
    {
      if(static_cast<double>(event.value) == 32767)
      {
        mc_rtc::log::info("[{}::NativeJoystick] Bottom pad", settings.name);
        if(ctl.datastore().has("ANA::ComTracking::Trigger"))
        {
          auto on_off_func = ctl.datastore().get<std::function<void(void)>>("ANA::ComTracking::Trigger");
          on_off_func();
        }
      }
      else if(static_cast<double>(event.value) == -32767) {}
    }
  }
}
