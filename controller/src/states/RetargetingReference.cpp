#include "RetargetingReference.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

void RetargetingReference::configure(const mc_rtc::Configuration & config)
{
  config("datastoreName_robot", datastoreNameRobot_);
  config("datastoreName_operator", datastoreNameOperator_);
  config("reference_frame_topic", reference_frame_operator_topic_);
  config("camera_pose_topic", camera_pose_topic_);
  config_.load(config);
}

void RetargetingReference::start(mc_control::fsm::Controller & ctl)
{
  if(!ctl.datastore().has(datastoreNameRobot_))
  {
    ctl.datastore().make<sva::PTransformd>(datastoreNameRobot_, sva::PTransformd::Identity());
  }
  if(!ctl.datastore().has(datastoreNameOperator_))
  {
    ctl.datastore().make<sva::PTransformd>(datastoreNameOperator_, sva::PTransformd::Identity());
  }
  ctl.datastore().make<bool>(datastoreNameOperator_ + "Active", false);

  if(!config_.has("robot") || (config_.has("robot") && !config_("robot").has(ctl.robot().name())))
  {

    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot {} defined in the YAML configuration", name(),
                                                     ctl.robot().name());
  }

  auto rConfig = config_("robot")(ctl.robot().name());
  rConfig("originOffset", originFrameOffset_);
  rConfig("torsoFrame", torsoFrame_);
  rConfig("camera_frame", camera_frame_);

  if(!ctl.robot().hasFrame(torsoFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No frame named \"{}\" for torso in robot \"{}\"", name(),
                                                     torsoFrame_, ctl.robot().name());
  }

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    operator_pose_sub_.subscribe(*nh_, reference_frame_operator_topic_);
    operator_pose_sub_.maxTime(0.5);

    camera_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(camera_pose_topic_, 1);
  }

  auto & gui = *ctl.gui();
  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Transform("Robot", [this]() -> const sva::PTransformd & { return X_0_referenceRobot_; }));
  gui.addElement(this, {"Avatar", "Unity", "Pose"},
                 mc_rtc::gui::Transform(
                     "Reference", [this]() -> const sva::PTransformd & { return X_0_referenceOperator_; },
                     [this](const sva::PTransformd & pose) { X_0_referenceOperator_ = pose; }));
  gui.addElement(this, {"Avatar", "Unity", "Data State"},
                 mc_rtc::gui::Checkbox(
                     "Reference", [this]() -> const bool & { return unity_online_; },
                     [this]() { unity_online_ = !unity_online_; }));

  gui.addElement(this, {"Avatar", name(), "Offsets"},

                 mc_rtc::gui::ArrayInput(
                     "Origin offset (translation) [m]", {"x", "y", "z"},
                     [this]() -> const Eigen::Vector3d & { return originFrameOffset_.translation(); },
                     [this](const Eigen::Vector3d & t) { originFrameOffset_.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "Origin offset (rotation) [deg]", {"r", "p", "y"},
                     [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(originFrameOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                     [this](const Eigen::Vector3d & rpy)
                     { originFrameOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));

  run(ctl);
  mc_rtc::log::info("[{}] started", name());
}

bool RetargetingReference::run(mc_control::fsm::Controller & ctl)
{
  const auto X_0_torso_link = ctl.robot().frame(torsoFrame_).position();
  const auto X_0_torso_reference = originFrameOffset_ * X_0_torso_link;

  X_0_referenceRobot_ = sva::PTransformd(originFrameOffset_.rotation())
                        * sva::PTransformd(stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                                               Eigen::Matrix3d::Identity(), X_0_torso_link.rotation()),
                                           X_0_torso_reference.translation());

  data_online_ = unity_online_;
  if(ctl.datastore().get<bool>("UseROS") && operator_pose_sub_.data().isValid())
  {
    X_0_referenceOperator_ = operator_pose_sub_.data().value();
    data_online_ = operator_pose_sub_.data().isValid();
  }

  sva::PTransformd X_0_camera = ctl.robot().frame(camera_frame_).position();

  sva::PTransformd X_ref_camera = X_0_camera * X_0_referenceRobot_.inv();

  if(ctl.datastore().get<bool>("UseROS"))
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = X_ref_camera.translation().x();
    pose.pose.position.y = X_ref_camera.translation().y();
    pose.pose.position.z = X_ref_camera.translation().z();
    Eigen::Quaterniond q{X_ref_camera.rotation().inverse()};
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    camera_pose_pub_.publish(pose);
  }

  ctl.datastore().get<sva::PTransformd>(datastoreNameRobot_) = X_0_referenceRobot_;
  ctl.datastore().get<sva::PTransformd>(datastoreNameOperator_) = X_0_referenceOperator_;
  ctl.datastore().get<bool>(datastoreNameOperator_ + "Active") = data_online_;

  output("OK");
  return true;
}

void RetargetingReference::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("RetargetingReference", RetargetingReference)
