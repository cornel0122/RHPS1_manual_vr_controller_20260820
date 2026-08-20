#include "common.h"

#include "../../ROSSubscriber.h"

void ROSJoyMsgControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto nh = ana_ros_node_handle();
  if(!nh) { return; }
  settings.available_modalities.push_back(to_string(Modality::ROSJoy));
  std::function<void(const sensor_msgs::Joy & msg)> callback = [this](const sensor_msgs::Joy & msg)
  {
    std::unique_lock<std::mutex> lck(data_mutex_);
    last_update_ = 0.0;
    data_[0] = msg.axes[forward_axis_];
    data_[1] = msg.axes[side_axis_];
    data_[2] = msg.axes[rot_axis_];
  };
  subscriber_ = nh->subscribe<sensor_msgs::Joy>(topic_, 10, callback);
}

void ROSJoyMsgControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  last_update_ += ctl.solver().dt();
  if(last_update_ > timeout_ && !walking.is_stopping())
  {
    mc_rtc::log::critical("[{}::ROSJoyMsg] No data received for {}s, stop walking now", settings.name, timeout_);
    walking.set_planner_ref_vel(Eigen::Vector3d::Zero());
    walking.start_stop_walking();
    return;
  }
  {
    std::unique_lock<std::mutex> lck(data_mutex_);
    double forward_input_ = deadzoned_input(data_[0], deadzone_.x());
    double side_input_ = deadzoned_input(data_[1], deadzone_.y());
    double rot_input_ = deadzoned_input(data_[2], deadzone_.z());
    if(forward_input_ > 0) { ref_vel_.x() = settings.max_vel_forward * forward_input_; }
    else { ref_vel_.x() = settings.max_vel_backward * abs(forward_input_); }
    ref_vel_.y() = settings.max_vel_side * side_input_;
    ref_vel_.z() = settings.max_vel_rot * rot_input_;
  }
  if(ref_vel_.norm() > 1e-6 && !walking.is_walking()) { walking.start_stop_walking(); }
  if(ref_vel_.norm() < 1e-6 && !walking.is_stopping()) { walking.start_stop_walking(); }
  walking.set_planner_ref_vel(ref_vel_);
}
