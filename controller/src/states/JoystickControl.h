#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/TransformTask.h>

#include "JoystickControl/Modality.h"

#include "../ROSSubscriber.h"
#include "../joystick/joystick.hh"

enum class WalkingDirection
{
  Forward,
  Backward,
  Side,
  TurnInPlace,
  Stand,
  None
};

struct JoystickControlCommonSettings
{
  std::string name;
  std::vector<std::string> available_modalities;
  std::string head_retargeting_name;
  double max_vel_forward = 0.2;
  double max_vel_backward = -0.2;
  double max_vel_side = 0.1;
  double max_vel_rot = 0.2;
};

struct NativeJoystickControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void handle_event(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  Joystick joystick;
  JoystickEvent event;
  bool connected;
  bool active = false;
  Eigen::Vector3d input_vel_ = Eigen::Vector3d::Zero();
  std::string left_hand_retargeting_name;
  std::string right_hand_retargeting_name;

  std::function<bool(void)> left_hand_on_func;
  std::function<bool(void)> right_hand_on_func;
};

struct ROSJoystickControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  bool active;

  std::string stride_trigger_topic_;
  size_t stride_trigger_count_ = 0;
  std::string forward_trigger_topic_;
  size_t forward_trigger_count_ = 0;
  std::string walking_dir_topic_;

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher walking_dir_pub_;
  ros::Publisher stride_trigger_pub_;
  ros::Publisher forward_trigger_pub_;
  ROSBoolSubscriber stride_trigger_sub_;
  ROSBoolSubscriber forward_trigger_sub_;
  WalkingDirection operator_walking_direction = WalkingDirection::Stand;
};

struct ROSJoyMsgControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  std::string topic_ = "/joy";
  size_t forward_axis_ = 1;
  size_t side_axis_ = 0;
  size_t rot_axis_ = 3;

  double timeout_ = 0.5;
  double last_update_ = 2 * timeout_;
  ros::Subscriber subscriber_;
  Eigen::Vector3d deadzone_ = {0.1, 0.1, 0.1};
  std::mutex data_mutex_;
  std::array<double, 3> data_ = {0, 0, 0};
  Eigen::Vector3d ref_vel_ = Eigen::Vector3d::Zero();
};

struct UnityHeadJoystickControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  std::string datastoreRetargetingReference_ = "RetargetingReferenceRobot";
  sva::PTransformd head_reference_frame_offset = sva::PTransformd::Identity();
  double min_rot_yaw_ = 10;
  double max_rot_yaw_ = 50;
  double min_vel_pitch_ = -5;
  double max_vel_pitch_ = 60;

  WalkingDirection operator_walking_direction = WalkingDirection::Forward;
  Eigen::Vector3d head_rpy;
  Eigen::Vector3d ref_vel_;
};

struct UnityJoystickControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  double timeout_ = 0.5;
  double last_update_ = 2 * timeout_;
  double forward_input_ = 0.0;
  double side_input_ = 0.0;
  double rot_input_ = 0.0;
  Eigen::Vector3d deadzone_ = {0.1, 0.1, 0.1};
  std::array<double, 3> data_ = {0, 0, 0};
  Eigen::Vector3d ref_vel_ = Eigen::Vector3d::Zero();
};

struct ManualSpeedControl
{
  void start(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  void run(mc_control::fsm::Controller & ctl, JoystickControlCommonSettings & settings);

  Eigen::Vector3d ref_vel_;
};

struct JoystickControl : mc_control::fsm::State
{

public:
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  JoystickControlCommonSettings settings;

  Modality modality_ = Modality::None;

  NativeJoystickControl native_joystick_;
  ROSJoystickControl ros_joystick_;
  ROSJoyMsgControl ros_joy_msg_joystick_;
  UnityHeadJoystickControl unity_head_joystick_;
  UnityJoystickControl unity_joystick_;
  ManualSpeedControl manual_speed_;

  bool walking_mode_enabled_ = false;
  std::vector<std::string> retargeting_state_names;
  void toggle_walking_mode(mc_control::fsm::Controller & ctl);
};
