#include <mc_control/ControllerServer.h>

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>

#include <mc_rtc/gui.h>

#include <chrono>
#include <thread>

struct UnityState
{
  void addToGUI(mc_rtc::gui::StateBuilder & gui)
  {
    gui.addElement({"Avatar", "Unity", "Trigger"}, mc_rtc::gui::Checkbox("ANA::SynchroWalking", trigger_synchrowalking),
                   mc_rtc::gui::Checkbox("ANA::HeadRetargeting", trigger_head_retargeting),
                   mc_rtc::gui::Checkbox("ANA::Head", trigger_head),
                   mc_rtc::gui::Checkbox("ANA::LeftHandRetargeting", trigger_left_hand_retargeting),
                   mc_rtc::gui::Checkbox("ANA::LeftArmRetargeting", trigger_left_arm_retargeting),
                   mc_rtc::gui::Checkbox("ANA::LeftRetargeting", trigger_left_retargeting),
                   mc_rtc::gui::Checkbox("ANA::RightHandRetargeting", trigger_right_hand_retargeting),
                   mc_rtc::gui::Checkbox("ANA::RightArmRetargeting", trigger_right_arm_retargeting),
                   mc_rtc::gui::Checkbox("ANA::RightRetargeting", trigger_right_retargeting));
    gui.addElement({"Avatar", "Unity", "Data State"}, mc_rtc::gui::Checkbox("Reference", data_state_reference),
                   mc_rtc::gui::Checkbox("ANA::HeadRetargeting", data_state_head),
                   mc_rtc::gui::Checkbox("ANA::LeftHandRetargeting", data_state_left_hand_retargeting),
                   mc_rtc::gui::Checkbox("ANA::LeftArmRetargeting", data_state_left_arm_retargeting),
                   mc_rtc::gui::Checkbox("ANA::RightHandRetargeting", data_state_right_hand_retargeting),
                   mc_rtc::gui::Checkbox("ANA::RightArmRetargeting", data_state_right_arm_retargeting));
    gui.addElement({"Avatar", "Unity", "Grippers"}, mc_rtc::gui::ArrayInput("ANA::LeftGripper", left_gripper),
                   mc_rtc::gui::ArrayInput("ANA::RightGripper", right_gripper));
    gui.addElement({"Avatar", "Unity", "Pose"}, mc_rtc::gui::Transform("Reference", pose_reference),
                   mc_rtc::gui::Transform("ANA::HeadRetargeting", pose_head),
                   mc_rtc::gui::Transform("ANA::LeftHandRetargeting", pose_left_hand),
                   mc_rtc::gui::Transform("ANA::LeftArmRetargeting", pose_left_arm),
                   mc_rtc::gui::Transform(
                       "ANA::RightHandRetargeting", [this]() { return pose_right_hand; },
                       [this](const sva::PTransformd & rh)
                       {
                         right_hand_updates++;
                         distance_from_last_right_hand = (rh * pose_right_hand.inv()).translation().norm();
                         pose_right_hand = rh;
                       }),
                   mc_rtc::gui::Transform("ANA::RightArmRetargeting", pose_right_arm));
    gui.addElement({"Avatar", "Unity", "Velocity"}, mc_rtc::gui::ArrayInput("ANA::HeadRetargeting", vel_head),
                   mc_rtc::gui::ArrayInput("ANA::LeftHandRetargeting", vel_left_hand),
                   mc_rtc::gui::ArrayInput("ANA::LeftArmRetargeting", vel_left_arm),
                   mc_rtc::gui::ArrayInput("ANA::RightHandRetargeting", vel_right_hand),
                   mc_rtc::gui::ArrayInput("ANA::RightArmRetargeting", vel_right_arm));
    gui.addElement({"Avatar", "Unity", "Joystick"}, mc_rtc::gui::ArrayInput("Stick", unity_joystick));
    gui.addElement({"Avatar", "Unity", "Walking"}, mc_rtc::gui::ArrayInput("ANA::SynchroWalkingLeft", walking_left),
                   mc_rtc::gui::ArrayInput("ANA::SynchroWalkingRight", walking_right));
  }

  void addToLog(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("Right hand online", [this]() { return data_state_right_hand_retargeting; });
    logger.addLogEntry("Distance from ref", [this]() { return distance_from_ref; });
    logger.addLogEntry("Change in distance from ref", [this]() { return change_in_distance_from_ref; });
    logger.addLogEntry("Distance between two iterations of right hand",
                       [this]() { return distance_from_last_right_hand; });
    logger.addLogEntry("Number of right hand updates", [this]() { return right_hand_updates; });
  }

  void tick()
  {
    right_hand_updates = 0;
    double ndistance = (pose_right_hand * pose_reference.inv()).translation().norm();
    change_in_distance_from_ref = std::abs(ndistance - distance_from_ref);
    distance_from_ref = ndistance;
  }

  bool trigger_synchrowalking = false;
  bool trigger_head_retargeting = false;
  bool trigger_head = false;
  bool trigger_left_hand_retargeting = false;
  bool trigger_left_arm_retargeting = false;
  bool trigger_left_retargeting = false;
  bool trigger_right_hand_retargeting = false;
  bool trigger_right_arm_retargeting = false;
  bool trigger_right_retargeting = false;

  bool data_state_reference = false;
  bool data_state_head = false;
  bool data_state_left_hand_retargeting = false;
  bool data_state_left_arm_retargeting = false;
  bool data_state_right_hand_retargeting = false;
  bool data_state_right_arm_retargeting = false;

  std::vector<double> left_gripper = {0.5};
  std::vector<double> right_gripper = {0.5};

  sva::PTransformd pose_reference = sva::PTransformd::Identity();
  sva::PTransformd pose_head = sva::PTransformd::Identity();
  sva::PTransformd pose_left_hand = sva::PTransformd::Identity();
  sva::PTransformd pose_left_arm = sva::PTransformd::Identity();
  sva::PTransformd pose_right_hand = sva::PTransformd::Identity();
  sva::PTransformd pose_right_arm = sva::PTransformd::Identity();

  sva::MotionVecd vel_head = sva::MotionVecd::Zero();
  sva::MotionVecd vel_left_hand = sva::MotionVecd::Zero();
  sva::MotionVecd vel_left_arm = sva::MotionVecd::Zero();
  sva::MotionVecd vel_right_hand = sva::MotionVecd::Zero();
  sva::MotionVecd vel_right_arm = sva::MotionVecd::Zero();
  sva::MotionVecd walking_left = sva::MotionVecd::Zero();
  sva::MotionVecd walking_right = sva::MotionVecd::Zero();

  Eigen::Vector3d unity_joystick = Eigen::Vector3d::Zero();

  size_t right_hand_updates = 0;
  double distance_from_last_right_hand = 0.0;
  double distance_from_ref = 0.0;
  double change_in_distance_from_ref = 0.0;
};

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    mc_rtc::log::error("Usage: {} [.bin]", argv[0]);
    return 1;
  }
  mc_rtc::log::FlatLog log(argv[1]);
  if(!log.meta())
  {
    mc_rtc::log::error("No meta info in {}", argv[1]);
    return 1;
  }
  const auto & meta = *log.meta();
  mc_control::ControllerServer server{
      meta.timestep, meta.timestep, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}};
  mc_rtc::gui::StateBuilder gui;
  mc_rtc::Logger logger(mc_rtc::Logger::Policy::THREADED, "/tmp", "debug");
  logger.start("unity", meta.timestep);

  bool quit = false;
  gui.addElement({}, mc_rtc::gui::Button("Quit", [&quit]() { quit = true; }));
  size_t iter = 0;
  gui.addElement({}, mc_rtc::gui::NumberSlider(
                         "Time", [&]() { return meta.timestep * iter; },
                         [&](double t) { iter = std::floor(t / meta.timestep); }, 0.0, meta.timestep * log.size()));
  size_t unity_update_iter = 0;
  logger.addLogEntry("Number of updates", [&]() { return unity_update_iter; });
  gui.addElement({}, mc_rtc::gui::NumberInput(
                         "Unity updates in iteration", [&]() { return unity_update_iter; }, [](double) {}));
  UnityState state;
  state.addToGUI(gui);
  state.addToLog(logger);
  while(iter < log.size())
  {
    auto now = std::chrono::high_resolution_clock::now();
    unity_update_iter = 0;
    server.handle_requests(gui);
    state.tick();
    if(iter < log.size())
    {
      const auto & events = log.guiEvents()[iter];
      for(const auto & e : events)
      {
        if(e.category.size() >= 2 && e.category[0] == "Avatar" && e.category[1] == "Unity")
        {
          unity_update_iter++;
          gui.handleRequest(e.category, e.name, e.data);
        }
      }
    }
    server.publish(gui);
    logger.log();
    iter++;
    // if(iter < log.size()) { iter++; }
    // std::this_thread::sleep_until(now + std::chrono::milliseconds(static_cast<size_t>(1000 * meta.timestep)));
  }

  return 0;
}
