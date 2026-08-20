#pragma once

#include <lipm_walking/Controller.h>

#include "WalkingInterface.h"
#include "api.h"

template<typename WalkingCtl>
struct ANAAvatarController_DLLAPI ANAAvatarController : public WalkingCtl
{

  ANAAvatarController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  void create_collision_cstr(const mc_rtc::Configuration & config);
  void reset(const mc_control::ControllerResetData & reset_data) override;
  bool run() override;

private:
  bool useROS_ = false; // Use ROS topics to retreive datas
  std::vector<std::string> safety_joints_ = {};
  double joint_safety_threshold_ = 0.05;
  bool emergency_posture_enabled_ = false;
  mc_avatar::WalkingInterfacePtr walking_interface_;
};

extern template struct ANAAvatarController<lipm_walking::Controller>;
extern template struct ANAAvatarController<mc_control::fsm::Controller>;
