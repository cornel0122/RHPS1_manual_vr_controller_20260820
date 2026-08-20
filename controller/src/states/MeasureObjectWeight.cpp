#include "MeasureObjectWeight.h"

void MeasureObjectWeight::configure(const mc_rtc::Configuration & config)
{
  config("pub_topic", pub_topic_);
  config("ForceSensorName", forceSensorName_);
  config("Frame", frame_name_);
  config("Contact_Frame", contact_frame_);
  config("finger_force_topic", finger_force_topic_);
  config("contact_ditance", contact_threshold_);
}

void MeasureObjectWeight::start(mc_control::fsm::Controller & ctl)
{
  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    weight_pub_ = nh_->advertise<std_msgs::Float64>(pub_topic_, 1);
    force_pub_ = nh_->advertise<geometry_msgs::Wrench>(finger_force_topic_, 1);
  }

  if(!ctl.robot().hasForceSensor(forceSensorName_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] robot {} has no sensor {}", name(), ctl.robot().name(),
                                                     forceSensorName_);
  }
  offset_ = ctl.robot().forceSensor(forceSensorName_).force().norm();

  auto & gui = *ctl.gui();
  gui.addElement(this, {"Avatar", "Unity", "State"},
                 mc_rtc::gui::ArrayInput(
                     name() + "Wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ForceVecd & { return measured_wrench_; }, [this](sva::ForceVecd f) {}),
                 mc_rtc::gui::ArrayInput(
                     name() + "Weight", {"w"}, [this]() { return std::vector<double>{weight_}; },
                     [this](std::vector<double> v) {}));

  output("OK");
}

bool MeasureObjectWeight::run(mc_control::fsm::Controller & ctl)
{

  auto sensor = ctl.robot().frame(frame_name_).forceSensor();
  Eigen::Vector3d frame_pose = ctl.robot().frame(frame_name_).position().translation();
  Eigen::Vector3d contact_frame_pose = ctl.robot().frame(contact_frame_).position().translation();

  measured_wrench_ = sensor.worldWrenchWithoutGravity(ctl.robot());
  weight_ = measured_wrench_.force().norm() - offset_;

  if((frame_pose - contact_frame_pose).norm() < contact_threshold_)
  {
    weight_ = (sensor.force() + ctl.robot().frame(contact_frame_).forceSensor().force()).norm();
  }

  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Float64 msg;
    geometry_msgs::Wrench msg_finger;
    msg_finger.force.x = measured_wrench_.force().x();
    msg_finger.force.z = measured_wrench_.force().y();
    msg_finger.force.y = measured_wrench_.force().z();
    msg_finger.torque.x = 0.;
    msg_finger.torque.y = 0.;
    msg_finger.torque.z = 0.;
    msg.data = weight_;
    weight_pub_.publish(msg);
    force_pub_.publish(msg_finger);
  }

  return true;
}

void MeasureObjectWeight::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("MeasureObjectWeight", MeasureObjectWeight)
