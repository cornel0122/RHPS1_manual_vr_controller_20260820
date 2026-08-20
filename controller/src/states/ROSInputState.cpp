#include "ROSInputState.h"
#include <mc_rtc/io_utils.h>
#include "../ANAAvatarController.h"

void ROSInputState::configure(const mc_rtc::Configuration & config)
{

  config("HandsJoyTopic", hands_joy_topic_);
  config("ROS_frequency", ros_frequency);
  config("EmergencyButtonTopic", emergency_button_topic_);
  config("TrackpadTopic", trackpad_topic_);
  config("EmergencyButtonMaxTime", emergency_button_max_time_);
  config("MaxPoseTime", maxTime_);
  config("simulation", simulation_);
}

void ROSInputState::start(mc_control::fsm::Controller & ctl)
{
  simulation_ = simulation_ || ctl.config()("simulation", false);

  nh_ = ana_ros_node_handle();
  if(!nh_) { mc_rtc::log::error_and_throw("ROS must be running for this controller to work"); }
  spinThread_ = std::thread(std::bind(&ROSInputState::rosSpinner, this));

  mc_rtc::log::info("[{}] Subscribing to {}", name(), hands_joy_topic_);
  mc_rtc::log::info("[{}] Subscribing to {}", name(), emergency_button_topic_);
  mc_rtc::log::info("[{}] Subscribing to {}", name(), trackpad_topic_);

  ps4_joy_sub_.subscribe(*nh_, "/joy");
  ps4_joy_sub_.maxTime(maxTime_);

  emergency_button_sub_.subscribe(*nh_, emergency_button_topic_);
  emergency_button_sub_.maxTime(emergency_button_max_time_);

  trackpad_sub_.subscribe(*nh_, trackpad_topic_);

  left_hand_joy_sub_.subscribe(*nh_, hands_joy_topic_);
  left_hand_joy_sub_.maxTime(maxTime_);
  right_hand_joy_sub_.subscribe(*nh_, hands_joy_topic_);
  right_hand_joy_sub_.maxTime(maxTime_);
  left_hand_joy_state_.dt(ctl.timeStep);
  right_hand_joy_state_.dt(ctl.timeStep);

  // ctl.datastore().make<SubscriberData<OcculusHandJoystick>>("LeftHandJoy");
  // ctl.datastore().make<SubscriberData<OcculusHandJoystick>>("RightHandJoy");

  if(!ctl.datastore().has("Emergency")) { ctl.datastore().make<bool>("Emergency", false); }
  ctl.datastore().make<SubscriberData<std::vector<float>>>("Trackpad");
  ctl.gui()->addElement(this, {}, mc_rtc::gui::ElementsStacking::Horizontal,
                        mc_rtc::gui::Button("Emergency STOP", [this]() { emergency_override_ = true; }),
                        mc_rtc::gui::Button("Resume", [this]() { emergency_override_ = false; }));

  ctl.gui()->addElement(
      this, {},
      mc_rtc::gui::Checkbox(
          "EMERGENCY ACTIVE", [&ctl]() { return ctl.datastore().get<bool>("Emergency"); }, [&ctl]() {}));

  // auto make_joystick_gui = [this, &ctl](OcculusHandJoystick & joystick, const OcculusHandJoystick & sjoy,
  //                                       const std::string & joystickName) {
  //   ctl.gui()->addElement(
  //       this, {"Avatar", "Occulus", joystickName},
  //       mc_rtc::gui::Checkbox(
  //           "X", [&joystick]() { return joystick.x; }, [&joystick]() { joystick.x = !joystick.x; }),
  //       mc_rtc::gui::Checkbox(
  //           "X clicked", [&sjoy]() { return sjoy.xClicked; }, []() {}),
  //       mc_rtc::gui::Checkbox(
  //           "Y", [&joystick]() { return joystick.y; }, [&joystick]() { joystick.y = !joystick.y; }),
  //       mc_rtc::gui::Checkbox(
  //           "Y clicked", [&sjoy]() { return sjoy.yClicked; }, []() {}),
  //       mc_rtc::gui::NumberSlider(
  //           "Primary trigger", [&joystick]() { return joystick.primary_trigger; },
  //           [&joystick](double val) { joystick.primary_trigger = val; }, 0, 1),
  //       mc_rtc::gui::Checkbox(
  //           "Primary trigger pressed", [&sjoy]() { return sjoy.primary_trigger_pressed; }, []() {}),
  //       mc_rtc::gui::Checkbox(
  //           "Primary trigger clicked", [&sjoy]() { return sjoy.primary_trigger_clicked; }, []() {}),
  //       mc_rtc::gui::NumberSlider(
  //           "Secondary trigger", [&joystick]() { return joystick.secondary_trigger; },
  //           [&joystick](double val) { joystick.secondary_trigger = val; }, 0, 1),
  //       mc_rtc::gui::Checkbox(
  //           "Secondary trigger pressed", [&sjoy]() { return sjoy.secondary_trigger_pressed; }, []() {}),
  //       mc_rtc::gui::Checkbox(
  //           "Secondary trigger clicked", [&sjoy]() { return sjoy.secondary_trigger_clicked; }, []() {}),
  //       mc_rtc::gui::NumberSlider(
  //           "Vertical", [&joystick]() { return joystick.vertical; },
  //           [&joystick](double val) { joystick.vertical = val; }, -1, 1),
  //       mc_rtc::gui::NumberSlider(
  //           "Horizontal", [&joystick]() { return joystick.horizontal; },
  //           [&joystick](double val) { joystick.horizontal = val; }, -1, 1));
  // };

  // make_joystick_gui(left_hand_joy_, left_hand_joy_state_.data(), "LeftHandJoy");
  // make_joystick_gui(right_hand_joy_, right_hand_joy_state_.data(), "RightHandJoy");
  output("OK");
}

void ROSInputState::rosSpinner()
{
  mc_rtc::log::info("ROS spinner thread created");
  ros::Rate r(ros_frequency);
  while(ros::ok() && running_)
  {
    ros::spinOnce();
    r.sleep();
  }
  mc_rtc::log::info("ROS spinner destroyed");
}

bool ROSInputState::run(mc_control::fsm::Controller & ctl)
{
  if(!simulation_) { getInput(ctl); }
  else { simulateInput(ctl); }

  // XXX not very clean interface, unneeded extra copies
  auto leftHandState = left_hand_joy_sub_.data();
  if(leftHandState.isValid())
  { // use real joystick if the data is valid. otherwise joystick is controlled from the gui
    left_hand_joy_ = left_hand_joy_sub_.data().value();
  }
  left_hand_joy_state_.data(left_hand_joy_);
  leftHandState.value(left_hand_joy_state_.data());
  // ctl.datastore().assign("LeftHandJoy", leftHandState);

  auto rightHandState = right_hand_joy_sub_.data();
  if(rightHandState.isValid()) { right_hand_joy_ = right_hand_joy_sub_.data().value(); }
  right_hand_joy_state_.data(right_hand_joy_);
  rightHandState.value(right_hand_joy_state_.data());
  // ctl.datastore().assign("RightHandJoy", rightHandState);

  return true;
}

void ROSInputState::getInput(mc_control::fsm::Controller & ctl)
{

  ps4_joy_sub_.tick(ctl.timeStep);
  emergency_button_sub_.tick(ctl.timeStep);
  trackpad_sub_.tick(ctl.timeStep);

  // if(ps4_joy_sub_.data().value().circle)
  // {
  //   ctl.datastore().assign("Emergency", true);
  // }
  // if(ps4_joy_sub_.data().value().triangle)
  // {
  //   ctl.datastore().assign("Emergency", false);
  // }

  // Emergency button (true when pressed)
  ctl.datastore().assign("Emergency", false);
  if(emergency_override_) { ctl.datastore().assign("Emergency", true); }
  else
  {
    if(ctl.datastore().has("EmergencyButtonPlugin"))
    {
      ctl.datastore().assign("Emergency", ctl.datastore().call<bool>("EmergencyButtonPlugin::State"));
    }
    // if(emergency_button_sub_.data().isValid())
    // {
    //   if(!emergency_button_connected_)
    //   {
    //     mc_rtc::log::success("[{}] Emergency button connected", name());
    //     emergency_button_connected_ = true;
    //   }
    //   ctl.datastore().assign("Emergency", emergency_button_sub_.data().value());
    // }
    // else
    // {
    //   if(emergency_button_connected_)
    //   {
    //     if(ctl.config()("use_emergency_button", true))
    //     {
    //       mc_rtc::log::error("[{}] Emergency button not connected!", name());
    //       ctl.datastore().assign("Emergency", true);
    //     }
    //     else
    //     {
    //       mc_rtc::log::warning("[{}] Emergency button is ignored (because use_emergency_button: true)", name());
    //       ctl.datastore().assign("Emergency", false);
    //     }
    //     emergency_button_connected_ = false;
    //   }
    // }
  }
  ctl.datastore().assign("Trackpad", trackpad_sub_.data());
}

// XXX simulateInput is essentially dead code now and should probably be removed
// alltogether
void ROSInputState::simulateInput(mc_control::fsm::Controller & ctl)
{
  const auto X_0_lh = ctl.robot().frame(leftHandFrame_).position();
  const auto X_0_rh = ctl.robot().frame(rightHandFrame_).position();
  static auto X_0_occulus = ctl.robot().frame(headFrame_).position();
  // static Eigen::Vector3d initial = X_0_occulus.translation();
  // X_0_occulus.translation() = initial;
  static auto X_occulus_lh = X_0_lh * X_0_occulus.inv();
  static auto X_occulus_rh = X_0_rh * X_0_occulus.inv();

  SubscriberData<sva::PTransformd> lh;
  lh.value(X_occulus_lh);
  SubscriberData<sva::PTransformd> rh;
  rh.value(X_occulus_rh);
  SubscriberData<sva::PTransformd> occulus;
  occulus.value(X_0_occulus);
  ctl.datastore().assign("LeftHandPose", lh);
  ctl.datastore().assign("RightHandPose", rh);
  ctl.datastore().assign("HeadPose", occulus);
}

void ROSInputState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  running_ = false;
  spinThread_.join();
  ctl.datastore().remove("FaceData");
  // ctl.datastore().remove("LeftHandJoy");
  // ctl.datastore().remove("RightHandJoy");
  ctl.datastore().remove("Trackpad");
}

EXPORT_SINGLE_STATE("ROSInputState", ROSInputState)
