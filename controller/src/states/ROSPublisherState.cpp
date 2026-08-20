#include "ROSPublisherState.h"
#include <mc_rtc/io_utils.h>
#include "../ANAAvatarController.h"
#include "../ROSSubscriber.h"

void ROSPublisherState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("Unity"))
  {
    auto c = config("Unity");
    c("InitPoseTopic", unity_pose_topic_);
    c("HUDStringTopic", unity_string_topic_);
    c("HUDHeadTopic", unity_head_tracking_topic_);
    c("HUDLeftHandTopic", unity_left_tracking_topic_);
    c("HUDRightHandTopic", unity_right_tracking_topic_);
    c("HUDComTopic", unity_com_tracking_topic_);
    c("HUDWalkingTopic", unity_walking_topic_);
    c("HUDWeightTopic", unity_weight_topic_);
  }
  config("cameraFrame", cameraFrame_);
  config("emergency_state_topic", emergency_topic_);

  config("useDatastore", useDatastore_);
  config("datastoreFrame", datastoreFrame_);
  config("publishRate", rate_);
}

void ROSPublisherState::start(mc_control::fsm::Controller & ctl)
{
  skip_ = static_cast<unsigned int>(ceil(1 / (rate_ * ctl.timeStep)));
  nh_ = ana_ros_node_handle();
  unity_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(unity_pose_topic_, 1);
  unity_string_pub_ = nh_->advertise<std_msgs::String>(unity_string_topic_, 1);
  unity_head_pub_ = nh_->advertise<std_msgs::Bool>(unity_head_tracking_topic_, 1);
  unity_left_pub_ = nh_->advertise<std_msgs::Bool>(unity_left_tracking_topic_, 1);
  unity_right_pub_ = nh_->advertise<std_msgs::Bool>(unity_right_tracking_topic_, 1);
  unity_com_pub_ = nh_->advertise<std_msgs::Bool>(unity_com_tracking_topic_, 1);
  unity_walking_pub_ = nh_->advertise<std_msgs::Bool>(unity_walking_topic_, 1);
  unity_weight_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(unity_weight_topic_, 1);
  unity_left_hand_force_pub_ = nh_->advertise<geometry_msgs::WrenchStamped>(unity_left_hand_force_topic_, 1);
  unity_right_hand_force_pub_ = nh_->advertise<geometry_msgs::WrenchStamped>(unity_right_hand_force_topic_, 1);

  emergency_pub = nh_->advertise<std_msgs::Bool>(emergency_topic_, 1);

  if(useDatastore_ && !ctl.datastore().has(datastoreFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No subscribed pose \"{}\" on the datastore", name(),
                                                     datastoreFrame_);
  }
  else if(!ctl.robot().hasFrame(cameraFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No surface \"{}\" in robot {}", name(), cameraFrame_,
                                                     ctl.robot().name());
  }

  output("OK");
}

bool ROSPublisherState::run(mc_control::fsm::Controller & ctl)
{
  if(++seq_ % skip_) { return true; }
  auto pt2msg = [](const sva::PTransformd & pt)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = pt.translation().x();
    pose.pose.position.y = pt.translation().y();
    pose.pose.position.z = pt.translation().z();
    Eigen::Quaterniond q{pt.rotation().inverse()};
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    return pose;
  };
  if(useDatastore_)
  {
    auto headPose = ctl.datastore().get<SubscriberData<sva::PTransformd>>("HeadPoseWorld").value().inv();
    unity_pose_pub_.publish(pt2msg(headPose));
  }
  else { unity_pose_pub_.publish(pt2msg(ctl.robot().frame(cameraFrame_).position().inv())); }
  std_msgs::String s;
  if(ctl.datastore().get<bool>("Emergency")) { s.data = "EMERGENCY STOP"; }
  else { s.data = ctl.datastore().get<std::string>("ANA::HUD"); }
  unity_string_pub_.publish(s);

  if(ctl.datastore().has("Emergency"))
  {
    std_msgs::Bool emergency_msg;
    emergency_msg.data = ctl.datastore().get<bool>("Emergency");
    emergency_pub.publish(emergency_msg);
  }

  // if(ctl.datastore().has("ANA::HeadTracking"))
  // {
  //   std_msgs::Bool msg;
  //   msg.data = ctl.datastore().get<bool>("ANA::HeadTracking");
  //   unity_head_pub_.publish(msg);
  //   msg.data = ctl.datastore().get<bool>("ANA::LeftHandPoseTracking");
  //   unity_left_pub_.publish(msg);
  //   msg.data = ctl.datastore().get<bool>("ANA::RightHandPoseTracking");
  //   unity_right_pub_.publish(msg);
  //   msg.data = ctl.datastore().get<bool>("ANA::ComTracking");
  //   unity_com_pub_.publish(msg);
  //   // msg.data = !ctl.datastore().call<bool>("Walking::IsPaused");
  //   unity_walking_pub_.publish(msg);
  // }

  std::string datastoreName = "ANA::ObjectWeights";
  if(ctl.datastore().has(datastoreName))
  {
    std_msgs::Float32MultiArray array;
    array.data = ctl.datastore().get<std::vector<float>>(datastoreName);
    unity_weight_pub_.publish(array);
  }

  geometry_msgs::WrenchStamped force_msg;
  datastoreName = "ANA::LeftHand_Force";
  if(ctl.datastore().has(datastoreName))
  {
    auto leftHandForce_ = ctl.datastore().get<sva::ForceVecd>(datastoreName);
    force_msg.wrench.force.x = leftHandForce_.force().x();
    force_msg.wrench.force.y = leftHandForce_.force().y();
    force_msg.wrench.force.z = leftHandForce_.force().z();
    force_msg.wrench.torque.x = leftHandForce_.couple().x();
    force_msg.wrench.torque.y = leftHandForce_.couple().y();
    force_msg.wrench.torque.z = leftHandForce_.couple().z();
    unity_left_hand_force_pub_.publish(force_msg);
  }

  datastoreName = "ANA::RightHand_Force";
  if(ctl.datastore().has(datastoreName))
  {
    auto rightHandForce_ = ctl.datastore().get<sva::ForceVecd>(datastoreName);
    force_msg.wrench.force.x = rightHandForce_.force().x();
    force_msg.wrench.force.y = rightHandForce_.force().y();
    force_msg.wrench.force.z = rightHandForce_.force().z();
    force_msg.wrench.torque.x = rightHandForce_.couple().x();
    force_msg.wrench.torque.y = rightHandForce_.couple().y();
    force_msg.wrench.torque.z = rightHandForce_.couple().z();
    unity_right_hand_force_pub_.publish(force_msg);
  }

  return true;
}

void ROSPublisherState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("ROSPublisherState", ROSPublisherState)
