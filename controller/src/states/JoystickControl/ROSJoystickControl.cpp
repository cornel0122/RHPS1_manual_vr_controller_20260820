#include "common.h"

void ROSJoystickControl::start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  if(!ctl.datastore().get<bool>("UseROS")) { return; }
  nh_ = ana_ros_node_handle();
  if(!nh_) { return; }
  settings.available_modalities.push_back(to_string(Modality::ROS));
  stride_trigger_sub_.subscribe(*nh_, stride_trigger_topic_);
  forward_trigger_sub_.subscribe(*nh_, forward_trigger_topic_);
  walking_dir_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(walking_dir_topic_, 1);
  stride_trigger_pub_ = nh_->advertise<std_msgs::Bool>(stride_trigger_topic_, 1);
  forward_trigger_pub_ = nh_->advertise<std_msgs::Bool>(forward_trigger_topic_, 1);
}

void ROSJoystickControl::run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings)
{
  auto & walking = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  auto & synchro_active_func = ctl.datastore().get<std::function<bool(void)>>("synchro_walk::active");
  if(!synchro_active_func()) { operator_walking_direction = WalkingDirection::Stand; };

  if(stride_trigger_sub_.data().value())
  {
    if(stride_trigger_count_ * ctl.timeStep > 1)
    {
      operator_walking_direction = WalkingDirection::Side;
      mc_rtc::log::info("[{}] switching to side walk", settings.name);
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      stride_trigger_count_ = 0;
      stride_trigger_pub_.publish(trigger_msg);
    }
    stride_trigger_count_++;
  }
  else { stride_trigger_count_ = 0; }
  if(forward_trigger_sub_.data().value())
  {
    if(forward_trigger_count_ * ctl.timeStep > 1)
    {
      operator_walking_direction = WalkingDirection::Forward;
      mc_rtc::log::info("[{}] switching to forward walk", settings.name);
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      forward_trigger_count_ = 0;
      forward_trigger_pub_.publish(trigger_msg);
    }
    forward_trigger_count_++;
  }
  else { forward_trigger_count_ = 0; }

  auto ref_vel_ = walking.get_planner_ref_vel();
  std_msgs::Float32MultiArray walking_dir_msg;
  walking_dir_msg.data = {(float)ref_vel_.x(), (float)ref_vel_.y(), (float)ref_vel_.z()};
  walking_dir_pub_.publish(walking_dir_msg);
}
