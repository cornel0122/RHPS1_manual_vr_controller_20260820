#pragma once

#include <mc_rtc/logging.h>

enum class Modality
{
  // No joystick-like modality available
  None,
  // Joystick connected to the same machine as the controller
  Native,
  // ROS topics
  ROS,
  // ROS sensor_msgs/Joy topic
  ROSJoy,
  // Unity voice + head interface
  UnityHead,
  // Unity joystick
  UnityJoystick,
  // Manually input speed
  ManualSpeed
};

inline std::string to_string(Modality mod)
{
  switch(mod)
  {
    case Modality::None:
      return "None";
    case Modality::Native:
      return "Native";
    case Modality::ROS:
      return "ROS";
    case Modality::ROSJoy:
      return "ROSJoy";
    case Modality::UnityHead:
      return "UnityHead";
    case Modality::UnityJoystick:
      return "UnityJoystick";
    case Modality::ManualSpeed:
      return "ManualSpeed";
    default:
      return "Unknown";
  }
}

inline Modality from_string(const std::string & mod)
{
  if(mod == "Native") { return Modality::Native; }
  if(mod == "ROS") { return Modality::ROS; }
  if(mod == "ROSJoy") { return Modality::ROSJoy; }
  if(mod == "UnityHead") { return Modality::UnityHead; }
  if(mod == "UnityJoystick") { return Modality::UnityJoystick; }
  if(mod == "ManualSpeed") { return Modality::ManualSpeed; }
  if(mod != "None")
  {
    mc_rtc::log::warning("[Modality from_string] Unknown value \"{}\" given, defaulting to None", mod);
  }
  return Modality::None;
}
