#include "ANAAvatarController.h"

extern "C"
{

  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    CONTROLLER_CHECK_VERSION("ANAAvatarControllerRHPS1")
    names = {"ANAAvatarControllerRHPS1", "ANAAvatarControllerRHPS1_manual", "ANAAvatarControllerRHPS1_none"};
  }

  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }

  CONTROLLER_MODULE_API unsigned int create_args_required()
  {
    return 4;
  }

  CONTROLLER_MODULE_API mc_control::MCController * create(const std::string & name,
                                                          const mc_rbdyn::RobotModulePtr & robot,
                                                          const double & dt,
                                                          const mc_control::Configuration & conf)
  {
    if(name == "ANAAvatarControllerRHPS1")
    {
      return new ANAAvatarController<lipm_walking::Controller>(robot, dt, conf);
    }
    if(name == "ANAAvatarControllerRHPS1_manual")
    {
      return new ANAAvatarController<lipm_walking::Controller>(robot, dt, conf);
    }
    if(name == "ANAAvatarControllerRHPS1_none")
    {
      return new ANAAvatarController<mc_control::fsm::Controller>(robot, dt, conf);
    }
    mc_rtc::log::error("This library cannot create a controller named {}", name);
    return nullptr;
  }
}
