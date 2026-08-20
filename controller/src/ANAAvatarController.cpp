#include "ANAAvatarController.h"

#include "WalkingInterfaceImpl.h"

#include <mc_rtc/ros.h>

template<typename WalkingCtl>
ANAAvatarController<WalkingCtl>::ANAAvatarController(mc_rbdyn::RobotModulePtr rm,
                                                     double dt,
                                                     const mc_rtc::Configuration & _config)
: WalkingCtl(rm, dt, _config, mc_control::ControllerParameters{}.load_robot_config_into({}).overwrite_config(true))
{
  auto nh = mc_rtc::ROSBridge::get_node_handle();
  if(!nh) { mc_rtc::log::error_and_throw("ROS is not available but required by this controller"); }
  walking_interface_ = std::make_shared<WalkingInterfaceImpl<WalkingCtl>>(*this);
  this->datastore().template make<mc_avatar::WalkingInterfacePtr>("WalkingInterface", walking_interface_);

  auto & config = this->config();
  if(config.has("UseROS")) { useROS_ = config("UseROS"); }
  this->datastore().template make<bool>("UseROS", useROS_);

  if(config.has("collisions"))
  {
    if(config("collisions").has(this->robot().name()))
    {
      create_collision_cstr(config("collisions")(this->robot().name()));
    }
  }
  if(config.has("joint_safety"))
  {
    if(config("joint_safety").has(this->robot().name()))
    {
      safety_joints_ = config("joint_safety")(this->robot().name())("joints");
    }
    joint_safety_threshold_ = config("joint_safety")("error_threshold");
  }

  this->datastore().template make<std::string>("ANA::HUD", "ANA Avatar");

  mc_rtc::log::success("ANAAvatarController init done ");
}

template<typename WalkingCtl>
void ANAAvatarController<WalkingCtl>::create_collision_cstr(const mc_rtc::Configuration & config)
{
  std::vector<mc_rbdyn::Collision> collisions;
  std::vector<std::string> robot_bodies;
  double iDist = config("iDist");
  double sDist = config("sDist");
  double default_iDist = iDist;
  double default_sDist = sDist;
  int cstr_set_indx = 0;

  if(config.has("simplified_all_bodies")) { robot_bodies = config("simplified_all_bodies"); }
  else
  {
    for(const auto & bd : this->robot().module().mb.bodies()) { robot_bodies.push_back(bd.name()); }
  }

  std::vector<std::string> robot_all_bodies = robot_bodies;

  while(true)
  {
    if(!config.has("cstr_set_" + std::to_string(cstr_set_indx))) { break; }
    else
    {
      mc_rtc::log::info("adding set {}", cstr_set_indx);
      std::string cstr_set = "cstr_set_" + std::to_string(cstr_set_indx);
      if(config(cstr_set).has("iDist")) { iDist = config(cstr_set)("iDist"); }
      if(config(cstr_set).has("sDist")) { sDist = config(cstr_set)("sDist"); }
      std::vector<std::string> bodies_1 = config(cstr_set)("b1");
      if(bodies_1.size() == 0) { bodies_1 = robot_bodies; }
      std::vector<std::string> bodies_2 = config(cstr_set)("b2");
      if(bodies_2.size() == 0) { bodies_2 = robot_all_bodies; }
      std::vector<std::string> joints = config(cstr_set)("joints");
      for(auto it = joints.begin(); it != joints.end();)
      {
        if(!this->robot().hasJoint(*it))
        {
          mc_rtc::log::error("Discarding joint {} because it does not exist in {}", *it, this->robot().name());
          it = joints.erase(it);
        }
        else { ++it; }
      }

      for(auto bd1 = std::begin(bodies_1); bd1 != std::end(bodies_1); bd1++)
      {
        for(auto bd2 = std::begin(bodies_2); bd2 != std::end(bodies_2); bd2++)
        {
          if(*bd1 != *bd2)
          {
            if(!this->robot().hasBody(*bd1))
            {
              mc_rtc::log::error("Discarding collision with {} because it does not exist in {}", *bd1,
                                 this->robot().name());
            }
            else if(!this->robot().hasBody(*bd2))
            {
              mc_rtc::log::error("Discarding collision with {} because it does not exist in {}", *bd2,
                                 this->robot().name());
            }
            else { collisions.push_back(mc_rbdyn::Collision(*bd1, *bd2, iDist, sDist, 0., joints, {})); }
          }
        }
        robot_bodies.erase(std::remove(robot_bodies.begin(), robot_bodies.end(), *bd1), robot_bodies.end());
      }
    }
    iDist = default_iDist;
    sDist = default_sDist;
    cstr_set_indx += 1;
  }
  this->addCollisions(this->robot().name(), this->robot().name(), collisions);
  mc_rtc::log::success("Self Collisions setted");
}

template<typename WalkingCtl>
bool ANAAvatarController<WalkingCtl>::run()
{

  // XXX Temporary hack for drill holding
  // We check the command error on the left wrist joints, if it's higher than 0.05 we reset the control model accordingly
  bool need_fk = false;
  for(const auto & j : safety_joints_)
  {
    auto mbcIndex = this->robot().jointIndexByName(j);
    auto & qOut = this->robot().mbc().q[mbcIndex][0];
    const auto & rjo = this->robot().refJointOrder();
    auto refIndex = std::distance(rjo.begin(), std::find(rjo.begin(), rjo.end(), j));
    auto qIn = this->robot().encoderValues()[static_cast<size_t>(refIndex)];
    auto error = qOut - qIn;
    auto max_vel = this->robot().module().bounds()[3].at(j)[0];
    auto max_step = 0.5 * max_vel * this->timeStep;
    auto max_error = joint_safety_threshold_;
    if(std::fabs(error) < max_error) { continue; }
    if(error > 0)
    {
      qOut -= max_step;
      this->robot().mbc().alpha[mbcIndex][0] = -0.5 * max_vel;
    }
    else
    {
      qOut += max_step;
      this->robot().mbc().alpha[mbcIndex][0] = 0.5 * max_vel;
    }

    need_fk = true;
  }
  if(need_fk)
  {
    this->robot().forwardKinematics();
    this->robot().forwardVelocity();
  }

  if(this->datastore().has("emergency_flag"))
  {
    if(this->datastore().template get<bool>("emergency_flag")) { return false; }
  }

  if(this->datastore().template get<bool>("Emergency"))
  {
    if(!emergency_posture_enabled_ && walking_interface_->is_stopped())
    {
      emergency_posture_enabled_ = true;
      mc_rtc::log::critical("EMERGENCY TRIGGERED, BYE\n");
      while(this->solver().tasks().size()) { this->solver().removeTask(this->solver().tasks()[0]); }
      this->solver().addTask(std::make_shared<mc_tasks::PostureTask>(this->solver(), 0, 100, 1000));
    }
  }
  return WalkingCtl::run();
}

template<typename WalkingCtl>
void ANAAvatarController<WalkingCtl>::reset(const mc_control::ControllerResetData & reset_data)
{
  WalkingCtl::reset(reset_data);
  if(auto init_pos_cfg = this->config().find("robots", this->robot().module().name, "init_pos"))
  {
    this->robot().posW(init_pos_cfg->operator sva::PTransformd());
    this->realRobot().posW(init_pos_cfg->operator sva::PTransformd());
  }
  if(!this->datastore().has("Emergency")) { this->datastore().template make<bool>("Emergency", false); }
  if constexpr(std::is_same_v<WalkingCtl, lipm_walking::Controller>)
  {
    // Enable external wrenches in the stabilizer
    auto StabTask = this->stabilizer();
    std::vector<std::string> surfaces = this->config()("ExternalWrenches")("surfaces");
    std::vector<sva::ForceVecd> wrenches(surfaces.size(), sva::ForceVecd::Zero());
    std::vector<sva::MotionVecd> gains(surfaces.size(), sva::MotionVecd(Eigen::Vector6d::Ones()));
    StabTask->setExternalWrenches(surfaces, wrenches, gains);
  }
}

/** Explicit instanciation of the controllers */
template struct ANAAvatarController<lipm_walking::Controller>;
template struct ANAAvatarController<mc_control::fsm::Controller>;
