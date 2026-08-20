#include "FacialExpression.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include "../ROSSubscriber.h"

void FacialExpression::configure(const mc_rtc::Configuration & config)
{
  config("sub_topic", sub_topic_);
  config("activeJoints", activeJoints_);
  config("stiffnesses", stiffnesses_);
}

void FacialExpression::start(mc_control::fsm::Controller & ctl)
{
  face_data_.resize(FaceJoints::COUNT);
  if(activeJoints_.size() != FaceJoints::COUNT)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] activeJoints array size mismatch. (Is {}, should be {}.)",
                                                     name(), activeJoints_.size(), FaceJoints::COUNT);
  }
  for(auto jnt : activeJoints_)
  {
    if(!ctl.robot().hasJoint(jnt))
    {

      Has_face_joints = false;
      mc_rtc::log::warning("[{}] robot {} has no face joints, Facial expression won't be used", name(),
                           ctl.robot().name());
      break;
    }
  }

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    face_sub_.subscribe(*nh_, sub_topic_);
    face_sub_.maxTime(maxTime_);
  }

  if(Has_face_joints)
  {
    // get FSM posture task
    postureTask_ = ctl.getPostureTask(ctl.robot().name());

    // set stiffnesses of active joints only
    std::vector<tasks::qp::JointStiffness> jnt_ss = {{"EYELID_P", stiffnesses_[jindex(FaceJoints::EYELID_P)]},
                                                     {"EYE_P", stiffnesses_[jindex(FaceJoints::EYE_P)]},
                                                     {"EYE_Y", stiffnesses_[jindex(FaceJoints::EYE_Y)]},
                                                     {"MOUTH_P", stiffnesses_[jindex(FaceJoints::MOUTH_P)]}};
    postureTask_->jointStiffness(ctl.solver(), jnt_ss);

    createGUI(ctl);
    mc_rtc::log::info("[{}] Started FacialExpression on {} joints.", name(), mc_rtc::io::to_string(activeJoints_));
  }
}

void FacialExpression::setJointTargetPercent(mc_control::fsm::Controller & ctl,
                                             const std::string & jnt_name,
                                             double tar)
{
  double jnt_llimit = ctl.robot().module().bounds()[0].find(jnt_name)->second[0];
  double jnt_ulimit = ctl.robot().module().bounds()[1].find(jnt_name)->second[0];
  mc_filter::utils::clampInPlace(tar, 0, 1);
  if(active_)
  {
    double target_ja = jnt_llimit + tar * (jnt_ulimit - jnt_llimit);
    postureTask_->target({{jnt_name, std::vector<double>{target_ja}}});
  }
}

void FacialExpression::setNominal(mc_control::fsm::Controller & ctl)
{
  const auto & halfSitPose = ctl.robot().module().stance();
  postureTask_->target({{jname(FaceJoints::EYE_P), halfSitPose.at(jname(FaceJoints::EYE_P))}});
  postureTask_->target({{jname(FaceJoints::EYE_Y), halfSitPose.at(jname(FaceJoints::EYE_Y))}});
  postureTask_->target({{jname(FaceJoints::MOUTH_P), halfSitPose.at(jname(FaceJoints::MOUTH_P))}});
}

bool FacialExpression::run(mc_control::fsm::Controller & ctl)
{
  if(!Has_face_joints) { return true; }

  bool online = face_data_online_;
  if(ctl.datastore().get<bool>("UseROS"))
  {
    // get latest message from ROS subscriber
    face_sub_.tick(ctl.solver().dt());
    face_data_ = std::vector<double>(face_sub_.data().value().begin(), face_sub_.data().value().end());
    online = face_sub_.data().isValid();
  }

  // control eyelid periodically
  int blink_len = period_ / ctl.timeStep;
  double eyelid_target = 0;
  if(ticks_ < (blink_len / 2)) { eyelid_target = 2 * (double(ticks_) / blink_len); }
  else if((blink_len / 2) <= ticks_ < blink_len) { eyelid_target = 2 - 2 * (double(ticks_) / blink_len); }
  setJointTargetPercent(ctl, jname(FaceJoints::EYELID_P), eyelid_target);

  if(online)
  {
    if(face_data_.size() == activeJoints_.size())
    {
      // set postureTask target
      // bool eyelid_target = face_data[0];
      float eye_p_target = face_data_[jindex(FaceJoints::EYE_P)];
      float eye_y_target = face_data_[jindex(FaceJoints::EYE_Y)];
      float mouth_target = face_data_[jindex(FaceJoints::MOUTH_P)];

      setJointTargetPercent(ctl, jname(FaceJoints::EYE_P), eye_p_target);
      setJointTargetPercent(ctl, jname(FaceJoints::EYE_Y), eye_y_target);
      setJointTargetPercent(ctl, jname(FaceJoints::MOUTH_P), mouth_target);
      size_warning_on_ = false;
    }
    else
    {
      if(!size_warning_on_)
      {
        mc_rtc::log::warning("[{}] Invalid data size received. Received: {}, but expected: {}. "
                             "Face will not move.",
                             name(), face_data_.size(), activeJoints_.size());
        size_warning_on_ = true;
      }
    }
  }
  else
  {
    // setNominal(ctl);
  }

  // counter update
  ticks_++;
  if(ticks_ >= (interval_ / ctl.timeStep)) { ticks_ = 0; }

  output("OK");
  return true;
}

void FacialExpression::teardown(mc_control::fsm::Controller & ctl)
{
  setNominal(ctl);
  ctl.gui()->removeElements(this);
}

void FacialExpression::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Label("Data online", [this]() { return face_sub_.data().isValid(); }),
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; }, [this]() { active_ = !active_; }));
  gui.addElement(this, {"Avatar", name(), "Task"},
                 mc_rtc::gui::NumberInput(
                     "EYELID_P_Stiffness", [this]() { return stiffnesses_[jindex(FaceJoints::EYELID_P)]; },
                     [this, &ctl](double s)
                     {
                       stiffnesses_[jindex(FaceJoints::EYELID_P)] = s;
                       postureTask_->jointStiffness(ctl.solver(), {{"EYELID_P", s}});
                     }),
                 mc_rtc::gui::NumberInput(
                     "EYE_P_Stiffness", [this]() { return stiffnesses_[jindex(FaceJoints::EYE_P)]; },
                     [this, &ctl](double s)
                     {
                       stiffnesses_[jindex(FaceJoints::EYE_P)] = s;
                       postureTask_->jointStiffness(ctl.solver(), {{"EYE_P", s}});
                     }),
                 mc_rtc::gui::NumberInput(
                     "EYE_Y_Stiffness", [this]() { return stiffnesses_[jindex(FaceJoints::EYE_Y)]; },
                     [this, &ctl](double s)
                     {
                       stiffnesses_[jindex(FaceJoints::EYE_Y)] = s;
                       postureTask_->jointStiffness(ctl.solver(), {{"EYE_Y", s}});
                     }),
                 mc_rtc::gui::NumberInput(
                     "MOUTH_P_Stiffness", [this]() { return stiffnesses_[jindex(FaceJoints::MOUTH_P)]; },
                     [this, &ctl](double s)
                     {
                       stiffnesses_[jindex(FaceJoints::MOUTH_P)] = s;
                       postureTask_->jointStiffness(ctl.solver(), {{"MOUTH_P", s}});
                     }));

  gui.addElement({"Avatar", "Unity", "Face"},
                 mc_rtc::gui::Checkbox(
                     "Online", [this]() { return face_data_online_; }, []() {}),
                 mc_rtc::gui::ArrayInput(
                     "FaceData", [this]() -> const std::vector<double> & { return face_data_; },
                     [this](const std::vector<double> & v)
                     {
                       face_data_online_ = true;
                       face_data_ = v;
                     }));
}

EXPORT_SINGLE_STATE("FacialExpression", FacialExpression)
