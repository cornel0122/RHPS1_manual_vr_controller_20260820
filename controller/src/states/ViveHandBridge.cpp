#include "ViveHandBridge.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

void ViveHandBridge::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  config("left_serial", left_.serial);
  config("right_serial", right_.serial);
  config("left_topic", left_.topic);
  config("right_topic", right_.topic);
  config("left_arm_serial", leftArm_.serial);
  config("right_arm_serial", rightArm_.serial);
  config("waist_serial", waist_.serial);
  config("left_arm_topic", leftArm_.topic);
  config("right_arm_topic", rightArm_.topic);
  config("waist_topic", waist_.topic);
  config("left_task", leftTaskName_);
  config("right_task", rightTaskName_);
  config("left_arm_task", leftArmTaskName_);
  config("right_arm_task", rightArmTaskName_);
  config("head_task", headTaskName_);
  config("scale", scale_);
  if(config.has("axis_scale"))
  {
    axisScale_ = config("axis_scale");
  }
  if(config.has("hand_position_offset_body"))
  {
    handPositionOffsetBody_ = config("hand_position_offset_body");
  }
  if(config.has("waist_reference_offset_body"))
  {
    waistReferenceOffsetBody_ = config("waist_reference_offset_body");
  }
  if(config.has("waist_to_robot_position_forward"))
  {
    const Eigen::Vector3d row = config("waist_to_robot_position_forward");
    waistToRobotPosition_.row(0) = row.transpose();
  }
  if(config.has("waist_to_robot_position_left"))
  {
    const Eigen::Vector3d row = config("waist_to_robot_position_left");
    waistToRobotPosition_.row(1) = row.transpose();
  }
  if(config.has("waist_to_robot_position_up"))
  {
    const Eigen::Vector3d row = config("waist_to_robot_position_up");
    waistToRobotPosition_.row(2) = row.transpose();
  }
  config("neutral_body_calibration", neutralBodyCalibration_);
  auto loadMatrixRows = [&config](const std::string & prefix, Eigen::Matrix3d & matrix) {
    const char * suffixes[3] = {"_x", "_y", "_z"};
    for(int rowIndex = 0; rowIndex < 3; ++rowIndex)
    {
      const std::string key = prefix + suffixes[rowIndex];
      if(config.has(key))
      {
        const Eigen::Vector3d row = config(key);
        matrix.row(rowIndex) = row.transpose();
      }
    }
  };
  loadMatrixRows("neutral_body_from_waist", neutralBodyFromWaist_);
  loadMatrixRows("left_hand_neutral_correction", leftHandNeutralCorrection_);
  loadMatrixRows("right_hand_neutral_correction", rightHandNeutralCorrection_);
  loadMatrixRows("left_arm_neutral_correction", leftArmNeutralCorrection_);
  loadMatrixRows("right_arm_neutral_correction", rightArmNeutralCorrection_);
  if(config.has("direct_hand_orientation_axis_fix_x"))
  {
    const Eigen::Vector3d row = config("direct_hand_orientation_axis_fix_x");
    directHandOrientationAxisFix_.row(0) = row.transpose();
  }
  if(config.has("direct_hand_orientation_axis_fix_y"))
  {
    const Eigen::Vector3d row = config("direct_hand_orientation_axis_fix_y");
    directHandOrientationAxisFix_.row(1) = row.transpose();
  }
  if(config.has("direct_hand_orientation_axis_fix_z"))
  {
    const Eigen::Vector3d row = config("direct_hand_orientation_axis_fix_z");
    directHandOrientationAxisFix_.row(2) = row.transpose();
  }
  if(config.has("direct_hand_rotation_sign_fix"))
  {
    directHandRotationSignFix_ = config("direct_hand_rotation_sign_fix");
  }
  config("direct_hand_rotation_axis_mode", directHandRotationAxisMode_);
  if(config.has("left_controller_hand_offset"))
  {
    leftControllerHandOffset_ = config("left_controller_hand_offset");
  }
  if(config.has("right_controller_hand_offset"))
  {
    rightControllerHandOffset_ = config("right_controller_hand_offset");
  }
  config("wrist_pivot_compensation", wristPivotCompensation_);
  config("left_position_horizontal_fix", leftPositionHorizontalFix_);
  config("left_position_horizontal_mode", leftPositionHorizontalMode_);
  config("debug_hand_mapping", debugHandMapping_);
  config("debug_hand_mapping_interval", debugHandMappingInterval_);
  if(config.has("left_hand_orientation_offset_rpy_deg"))
  {
    leftHandOrientationCorrection_ = rpyDegreesToMatrix(config("left_hand_orientation_offset_rpy_deg"));
  }
  if(config.has("right_hand_orientation_offset_rpy_deg"))
  {
    rightHandOrientationCorrection_ = rpyDegreesToMatrix(config("right_hand_orientation_offset_rpy_deg"));
  }
  config("max_step", maxStep_);
  config("direct_waist_max_distance", directWaistMaxDistance_);
  config("input_delay", inputDelay_);
  config("experiment_condition_enabled", experimentConditionEnabled_);
  config("experiment_condition_required", experimentConditionRequired_);
  config("experiment_rearm_on_command", experimentRearmOnCommand_);
  config("experiment_condition_topic", experimentCommandTopic_);
  config("experiment_delayed_input_seconds", experimentDelayedInputSeconds_);
  experimentDelayedInputSeconds_ = std::max(0.0, experimentDelayedInputSeconds_);
  config("auto_zero", autoZero_);
  config("require_initial_two_hand_reset", requireInitialTwoHandReset_);
  config("initial_zero_requires_arm_trackers", initialZeroRequiresArmTrackers_);
  config("control_grippers", controlGrippers_);
  config("control_mujoco_adhesion", controlMujocoAdhesion_);
  config("control_hand_orientation", controlHandOrientation_);
  config("invert_hand_orientation_delta", invertHandOrientationDelta_);
  config("invert_arm_orientation_delta", invertArmOrientationDelta_);
  if(config.has("arm_rotation_sign_fix"))
  {
    armRotationSignFix_ = config("arm_rotation_sign_fix");
  }
  if(config.has("arm_angular_velocity_sign_fix"))
  {
    armAngularVelocitySignFix_ = config("arm_angular_velocity_sign_fix");
  }
  config("left_hand_rotation_axis_mode", leftHandRotationAxisMode_);
  config("right_hand_rotation_axis_mode", rightHandRotationAxisMode_);
  config("use_waist_reference", useWaistReference_);
  config("direct_waist_control", directWaistControl_);
  config("robot_waist_frame", robotWaistFrame_);
  config("direct_waist_keep_hand_side", directWaistKeepHandSide_);
  config("tracking_handover_duration", trackingHandoverDuration_);
  config("two_hand_press_window", twoHandPressWindow_);
  config("two_hand_pause_hold_duration", twoHandPauseHoldDuration_);
  config("experiment_reset_duration", experimentResetDuration_);
  twoHandPressWindow_ = std::max(0.05, twoHandPressWindow_);
  twoHandPauseHoldDuration_ = std::max(0.5, twoHandPauseHoldDuration_);
  experimentResetDuration_ = std::max(1.0, experimentResetDuration_);
  config("adhesion_topic", adhesionTopic_);
  config("grasp_event_topic", graspEventTopic_);
  if(config.has("grasp_notification_trigger_threshold"))
  {
    config("grasp_notification_trigger_threshold", graspNotificationTriggerThreshold_);
  }
  else
  {
    // 兼容旧配置名，但该阈值现在只用于 trigger 通知，不再控制夹具。
    config("gripper_trigger_threshold", graspNotificationTriggerThreshold_);
  }
  config("use_velocity_feedforward", useVelocityFeedforward_);
  config("velocity_feedforward_gain", velocityFeedforwardGain_);
  useLinearVelocityFeedforward_ = useVelocityFeedforward_;
  linearVelocityGain_ = velocityFeedforwardGain_;
  config("use_linear_velocity_feedforward", useLinearVelocityFeedforward_);
  config("use_angular_velocity_feedforward", useAngularVelocityFeedforward_);
  config("linear_velocity_gain", linearVelocityGain_);
  config("angular_velocity_gain", angularVelocityGain_);
  config("velocity_filter_alpha", velocityFilterAlpha_);
  config("velocity_sample_timeout", velocitySampleTimeout_);
  velocityFilterAlpha_ = std::clamp(velocityFilterAlpha_, 0.0, 1.0);
  velocitySampleTimeout_ = std::max(0.0, velocitySampleTimeout_);
  config("velocity_validation", velocityValidation_);
  config("velocity_validation_arrow_scale", velocityValidationArrowScale_);
  config("velocity_validation_min_speed", velocityValidationMinSpeed_);
  config("publish_legacy_unity_topics", publishLegacyUnityTopics_);
  config("left_gripper", leftGripper_);
  config("right_gripper", rightGripper_);
  config("left_gripper_joint", leftGripperJoint_);
  config("right_gripper_joint", rightGripperJoint_);
  config("gripper_profile", gripperProfile_);
  config("gripper_expected_active_joints", expectedGripperActiveJoints_);
  config("left_gripper_commands_enabled", leftGripperCommandsEnabled_);
  config("right_gripper_commands_enabled", rightGripperCommandsEnabled_);
  config("left_robot_hand_frame", leftRobotHandFrame_);
  config("right_robot_hand_frame", rightRobotHandFrame_);
  config("open_value", openValue_);
  config("closed_value", closedValue_);
  config("gripper_safety_enabled", gripperSafetyEnabled_);
  if(config.has("gripper_safety"))
  {
    const auto safety = config("gripper_safety");
    safety("threshold", gripperSafetyThresholdDeg_);
    safety("iter", gripperSafetyIterations_);
    safety("release", gripperSafetyReleaseDeg_);
    safety("vmaxPercent", gripperSafetyVmaxPercent_);
  }
  gripperSafetyIterations_ = std::max(1, gripperSafetyIterations_);
  gripperSafetyVmaxPercent_ = std::clamp(gripperSafetyVmaxPercent_, 0.0, 1.0);
  config("adhesion_open_value", adhesionOpenValue_);
  config("adhesion_closed_value", adhesionClosedValue_);
}

void ViveHandBridge::start(mc_control::fsm::Controller & ctl)
{
  // 共享自主通过这个只读状态判断 Vive 是否已经完成双手确认并开始控制。
  ctl.datastore().make_call(name() + "::tracking_started", [this]() -> bool { return trackingStarted_; });
  ctl.datastore().make_call("ANA::Vive::tracking_started", [this]() -> bool { return trackingStarted_; });

  if(experimentConditionEnabled_)
  {
    ctl.datastore().make<int>(experimentConditionKey_, experimentCondition_);
    ctl.datastore().make<std::string>(experimentConditionNameKey_, experimentConditionName_);
    ctl.datastore().make<std::string>(experimentSessionKey_, experimentSessionId_);
    ctl.datastore().make<bool>(experimentAssistanceKey_, false);
    ctl.datastore().make<double>(experimentDelayKey_, inputDelay_);
    ctl.datastore().make<bool>(experimentConditionReadyKey_, false);
  }

  // 机器人相关的 frame、夹具和人体相对偏移在启动时按当前 RobotModule 覆盖。
  // 这样 HRP4CR 与 RHPS1 可以共用 Vive 设备校准，同时保留各自的机器人参数。
  if(config_.has("robot") && config_("robot").has(ctl.robot().name()))
  {
    const auto robotConfig = config_("robot")(ctl.robot().name());
    robotConfig("robot_waist_frame", robotWaistFrame_);
    robotConfig("left_robot_hand_frame", leftRobotHandFrame_);
    robotConfig("right_robot_hand_frame", rightRobotHandFrame_);
    robotConfig("left_gripper", leftGripper_);
    robotConfig("right_gripper", rightGripper_);
    robotConfig("left_gripper_joint", leftGripperJoint_);
    robotConfig("right_gripper_joint", rightGripperJoint_);
    robotConfig("waist_reference_offset_body", waistReferenceOffsetBody_);
    robotConfig("hand_position_offset_body", handPositionOffsetBody_);
    robotConfig("control_mujoco_adhesion", controlMujocoAdhesion_);
    robotConfig("open_value", openValue_);
    robotConfig("closed_value", closedValue_);
  }

  if(controlGrippers_)
  {
    auto prepareGripper = [&](HandState & hand,
                              bool commandsEnabled,
                              const std::string & gripperName,
                              const std::string & configuredJoint,
                              const std::string & side) {
      hand.gripperReady = false;
      if(!commandsEnabled)
      {
        mc_rtc::log::info(
            "[{}] {} gripper commands are disabled by configuration; arm retargetting and Vive menu controls remain active",
            name(), side);
        return;
      }
      if(!ctl.robot().hasGripper(gripperName))
      {
        mc_rtc::log::error("[{}] {} gripper '{}' is not registered; Vive grip commands are disabled",
                           name(), side, gripperName);
        return;
      }
      auto & gripper = ctl.robot().gripper(gripperName);
      if(gripper.activeJoints().empty())
      {
        mc_rtc::log::error(
            "[{}] {} gripper '{}' has no active joints; Vive grip commands cannot move it",
            name(), side, gripperName);
        return;
      }
      if(expectedGripperActiveJoints_ > 0
         && gripper.activeJoints().size() != static_cast<size_t>(expectedGripperActiveJoints_))
      {
        mc_rtc::log::error(
            "[{}] {} gripper '{}' profile '{}' expects {} active joint(s), but RobotModule registered {}; "
            "Vive grip commands are disabled",
            name(), side, gripperName, gripperProfile_, expectedGripperActiveJoints_, gripper.activeJoints().size());
      }
      else
      {
        if(!gripper.hasActiveJoint(configuredJoint))
        {
          mc_rtc::log::info(
              "[{}] {} gripper '{}' does not contain legacy joint '{}'; whole-gripper commands will control "
              "all {} RobotModule active joints",
              name(), side, gripperName, configuredJoint, gripper.activeJoints().size());
        }
        if(gripperSafetyEnabled_)
        {
          gripper.actualCommandDiffTrigger(mc_rtc::constants::toRad(gripperSafetyThresholdDeg_));
          gripper.overCommandLimitIterN(static_cast<unsigned int>(gripperSafetyIterations_));
          gripper.releaseSafetyOffset(mc_rtc::constants::toRad(gripperSafetyReleaseDeg_));
          gripper.percentVMAX(gripperSafetyVmaxPercent_);
          mc_rtc::log::info(
              "[{}] {} gripper safety: threshold={:.1f}deg, iter={}, release={:.1f}deg, vmax={:.1f}%",
              name(), side, gripperSafetyThresholdDeg_, gripperSafetyIterations_, gripperSafetyReleaseDeg_,
              100.0 * gripperSafetyVmaxPercent_);
        }
        hand.gripperReady = true;
        mc_rtc::log::success(
            "[{}] {} gripper '{}' ready: profile={}, active_joints={}, Vive grip toggles open/close",
            name(), side, gripperName, gripperProfile_, gripper.activeJoints().size());
      }
    };
    prepareGripper(left_, leftGripperCommandsEnabled_, leftGripper_, leftGripperJoint_, "left");
    prepareGripper(right_, rightGripperCommandsEnabled_, rightGripper_, rightGripperJoint_, "right");
  }

  mc_rtc::log::info("[{}] ViveHandBridge loaded: neutral_body_calibration={}, wrist_pivot_compensation={}, "
                    "left_position_horizontal_fix={}, "
                    "left_position_horizontal_mode={}, linear_velocity_ff={}, angular_velocity_ff={}, "
                    "debug_hand_mapping={}, waist_offset_body=[{:.3f}, {:.3f}, {:.3f}], "
                    "hand_offset_body=[{:.3f}, {:.3f}, {:.3f}], handover={:.1f}s",
                    name(),
                    neutralBodyCalibration_,
                    wristPivotCompensation_,
                    leftPositionHorizontalFix_,
                    leftPositionHorizontalMode_,
                    useLinearVelocityFeedforward_,
                    useAngularVelocityFeedforward_,
                    debugHandMapping_,
                    waistReferenceOffsetBody_.x(),
                    waistReferenceOffsetBody_.y(),
                    waistReferenceOffsetBody_.z(),
                    handPositionOffsetBody_.x(),
                    handPositionOffsetBody_.y(),
                    handPositionOffsetBody_.z(),
                    trackingHandoverDuration_);

  if(!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    ownsRclcpp_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("ana_vive_hand_bridge");
  graspEventPub_ = node_->create_publisher<std_msgs::msg::String>(graspEventTopic_, 10);
  if(experimentConditionEnabled_)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    experimentCommandSub_ = node_->create_subscription<std_msgs::msg::String>(
        experimentCommandTopic_, qos,
        [this](const std_msgs::msg::String::SharedPtr msg) { onExperimentCommand(msg); });
    mc_rtc::log::info("[{}] Waiting for experiment condition 1-4 on {}", name(), experimentCommandTopic_);
  }
  if(controlMujocoAdhesion_)
  {
    // 磁力抓取只发送左右夹具的吸附强度，真正写入 MuJoCo actuator 的部分放在 mc_mujoco 里。
    adhesionPub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(adhesionTopic_, 1);
  }
  if(publishLegacyUnityTopics_)
  {
    // 额外发布 Guillaume 原版 Unity/ROS1 风格的话题，便于对比旧链路并逐步切回原始 retargetting 输入。
    left_.legacyPosePub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("avatar/left_hand/pose", 1);
    left_.legacyVelocityPub =
        node_->create_publisher<geometry_msgs::msg::AccelStamped>("avatar/left_hand/velocity", 1);
    right_.legacyPosePub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("avatar/right_hand/pose", 1);
    right_.legacyVelocityPub =
        node_->create_publisher<geometry_msgs::msg::AccelStamped>("avatar/right_hand/velocity", 1);
    leftArm_.legacyPosePub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("avatar/left_arm/pose", 1);
    leftArm_.legacyVelocityPub =
        node_->create_publisher<geometry_msgs::msg::AccelStamped>("avatar/left_arm/velocity", 1);
    rightArm_.legacyPosePub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("avatar/right_arm/pose", 1);
    rightArm_.legacyVelocityPub =
        node_->create_publisher<geometry_msgs::msg::AccelStamped>("avatar/right_arm/velocity", 1);
  }

  if(ctl.datastore().has(leftTaskName_))
  {
    left_.task = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(leftTaskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing datastore task {}", name(), leftTaskName_);
  }

  if(ctl.datastore().has(rightTaskName_))
  {
    right_.task = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(rightTaskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing datastore task {}", name(), rightTaskName_);
  }

  if(ctl.robot().hasFrame(robotWaistFrame_) && ctl.robot().hasFrame(leftRobotHandFrame_)
     && ctl.robot().hasFrame(rightRobotHandFrame_))
  {
    // ViveHandBridge 在 Guillaume 的 PutHandsAboveTable 之后启动。
    // 此时记录机器人标准工作姿势下，两只手相对 BODY 的精确锚点，供人体距离校准使用。
    const auto & X_0_body = ctl.robot().frame(robotWaistFrame_).position();
    const auto X_body_left = ctl.robot().frame(leftRobotHandFrame_).position() * X_0_body.inv();
    const auto X_body_right = ctl.robot().frame(rightRobotHandFrame_).position() * X_0_body.inv();
    mc_rtc::log::info("[{}] {} work anchors relative to {}: left=[{:.6f}, {:.6f}, {:.6f}], "
                      "right=[{:.6f}, {:.6f}, {:.6f}]",
                      name(),
                      ctl.robot().name(),
                      robotWaistFrame_,
                      X_body_left.translation().x(),
                      X_body_left.translation().y(),
                      X_body_left.translation().z(),
                      X_body_right.translation().x(),
                      X_body_right.translation().y(),
                      X_body_right.translation().z());
  }
  else
  {
    mc_rtc::log::warning("[{}] Cannot record work anchors: missing frame {}, {} or {}",
                         name(),
                         robotWaistFrame_,
                         leftRobotHandFrame_,
                         rightRobotHandFrame_);
  }

  if(ctl.datastore().has(leftArmTaskName_))
  {
    leftArm_.task = ctl.datastore().get<std::shared_ptr<mc_tasks::TransformTask>>(leftArmTaskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing datastore task {}", name(), leftArmTaskName_);
  }

  if(ctl.datastore().has(rightArmTaskName_))
  {
    rightArm_.task = ctl.datastore().get<std::shared_ptr<mc_tasks::TransformTask>>(rightArmTaskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing datastore task {}", name(), rightArmTaskName_);
  }

  if(ctl.datastore().has(headTaskName_))
  {
    headTask_ = ctl.datastore().get<std::shared_ptr<mc_tasks::TransformTask>>(headTaskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing optional datastore task {}; head will not be included in GUI reset",
                         name(), headTaskName_);
  }

  // 等待首次双手确认期间，四个 retargetting task 必须保持当前机器人姿态。
  // 否则任务内部残留的 Unity 目标会在 Vive 数据接管前先拉动手臂。
  if(left_.task)
  {
    left_.zeroRobot = left_.task->frame().position();
    left_.task->target(left_.zeroRobot);
    left_.task->targetVel(sva::MotionVecd::Zero());
  }
  if(right_.task)
  {
    right_.zeroRobot = right_.task->frame().position();
    right_.task->target(right_.zeroRobot);
    right_.task->targetVel(sva::MotionVecd::Zero());
  }
  if(leftArm_.task)
  {
    leftArm_.zeroRobot = leftArm_.task->frame().position();
    leftArm_.task->target(leftArm_.zeroRobot);
    leftArm_.task->targetVel(sva::MotionVecd::Zero());
  }
  if(rightArm_.task)
  {
    rightArm_.zeroRobot = rightArm_.task->frame().position();
    rightArm_.task->target(rightArm_.zeroRobot);
    rightArm_.task->targetVel(sva::MotionVecd::Zero());
  }

  captureExperimentStartPose(ctl);

  auto addUnique = [](std::vector<std::string> & topics, const std::string & topic) {
    if(!topic.empty() && std::find(topics.begin(), topics.end(), topic) == topics.end())
    {
      topics.push_back(topic);
    }
  };

  std::vector<std::string> controllerTopics;
  addUnique(controllerTopics, left_.topic);
  addUnique(controllerTopics, right_.topic);
  // 额外监听默认 controller topic，避免 SteamVR 重新配对后左右 topic 顺序变化。
  addUnique(controllerTopics, "/controller_1");
  addUnique(controllerTopics, "/controller_2");
  for(const auto & topic : controllerTopics)
  {
    controllerSubs_.push_back(node_->create_subscription<Controller>(
        topic, 10, [this, topic](const Controller::SharedPtr msg) { onAnyController(topic, msg); }));
  }

  std::vector<std::string> trackerTopics;
  addUnique(trackerTopics, leftArm_.topic);
  addUnique(trackerTopics, rightArm_.topic);
  addUnique(trackerTopics, waist_.topic);
  // 同时监听所有 tracker topic，再用 serial_number 匹配左肘、右肘和腰部。
  addUnique(trackerTopics, "/tracker_1");
  addUnique(trackerTopics, "/tracker_2");
  addUnique(trackerTopics, "/tracker_3");
  addUnique(trackerTopics, "/tracker_4");
  for(const auto & topic : trackerTopics)
  {
    trackerSubs_.push_back(
        node_->create_subscription<Tracker>(topic, 10, [this, topic](const Tracker::SharedPtr msg) { onAnyTracker(topic, msg); }));
  }

  ctl.gui()->addElement(
      this, {"Avatar", "Vive"},
      mc_rtc::gui::Label("Control state", [this]() { return controlStateName(); }),
      mc_rtc::gui::Label("Tracking started", [this]() { return trackingStarted_; }),
      mc_rtc::gui::Label("Both-menu hold [s]", [this]() { return bothMenuHoldProgress_; }),
      mc_rtc::gui::Label("Start/resume count", [this]() { return std::to_string(startResumeCount_); }),
      mc_rtc::gui::Label("Pause count", [this]() { return std::to_string(pauseCount_); }),
      mc_rtc::gui::Label("Experiment condition received", [this]() { return experimentConditionReceived_; }),
      mc_rtc::gui::Label("Experiment condition", [this]() -> const std::string & {
        return experimentConditionName_;
      }),
      mc_rtc::gui::Label("Experiment session", [this]() -> const std::string & {
        return experimentSessionId_;
      }),
      mc_rtc::gui::Label("Experiment assistance enabled", [this]() { return experimentAssistanceEnabled_; }),
      mc_rtc::gui::Label("Initial left reset", [this]() { return initialLeftResetSeen_; }),
      mc_rtc::gui::Label("Initial right reset", [this]() { return initialRightResetSeen_; }),
      mc_rtc::gui::Label("Left data", [this]() { return left_.hasData; }),
      mc_rtc::gui::Label("Right data", [this]() { return right_.hasData; }),
      mc_rtc::gui::Label("Left menu raw", [this]() { return left_.rawResetButton; }),
      mc_rtc::gui::Label("Right menu raw", [this]() { return right_.rawResetButton; }),
      mc_rtc::gui::Label("Left menu press count", [this]() { return std::to_string(left_.resetPressCount); }),
      mc_rtc::gui::Label("Right menu press count", [this]() { return std::to_string(right_.resetPressCount); }),
      mc_rtc::gui::Label("Left zeroed", [this]() { return left_.zeroed; }),
      mc_rtc::gui::Label("Right zeroed", [this]() { return right_.zeroed; }),
      mc_rtc::gui::Label("Left arm tracker", [this]() { return leftArm_.hasData; }),
      mc_rtc::gui::Label("Right arm tracker", [this]() { return rightArm_.hasData; }),
      mc_rtc::gui::Label("Waist tracker", [this]() { return waist_.hasData; }),
      mc_rtc::gui::Label("Left gripper closed", [this]() { return left_.gripperClosed; }),
      mc_rtc::gui::Label("Right gripper closed", [this]() { return right_.gripperClosed; }),
      mc_rtc::gui::Label("MuJoCo adhesion", [this]() { return controlMujocoAdhesion_; }),
      mc_rtc::gui::Label("Waist reference", [this]() { return useWaistReference_ && waist_.hasData; }),
      mc_rtc::gui::Label("Direct waist control", [this]() { return directWaistControl_; }),
      mc_rtc::gui::Label("Neutral body calibration", [this]() { return neutralBodyCalibration_; }),
      mc_rtc::gui::Label("Wrist pivot compensation", [this]() { return wristPivotCompensation_; }),
      mc_rtc::gui::NumberInput("Scale", [this]() { return scale_; }, [this](double s) { scale_ = s; }),
      mc_rtc::gui::NumberInput("Forward scale", [this]() { return axisScale_.x(); },
                               [this](double s) { axisScale_.x() = s; }),
      mc_rtc::gui::NumberInput("Lateral scale", [this]() { return axisScale_.y(); },
                               [this](double s) { axisScale_.y() = s; }),
      mc_rtc::gui::NumberInput("Vertical scale", [this]() { return axisScale_.z(); },
                               [this](double s) { axisScale_.z() = s; }),
      mc_rtc::gui::NumberInput("Waist forward offset [m]", [this]() { return waistReferenceOffsetBody_.x(); },
                               [this](double offset) { waistReferenceOffsetBody_.x() = offset; }),
      mc_rtc::gui::NumberInput("Waist lateral offset [m]", [this]() { return waistReferenceOffsetBody_.y(); },
                               [this](double offset) { waistReferenceOffsetBody_.y() = offset; }),
      mc_rtc::gui::NumberInput("Waist vertical offset [m]", [this]() { return waistReferenceOffsetBody_.z(); },
                               [this](double offset) { waistReferenceOffsetBody_.z() = offset; }),
      mc_rtc::gui::NumberInput("Hand vertical offset [m]", [this]() { return handPositionOffsetBody_.z(); },
                               [this](double offset) { handPositionOffsetBody_.z() = offset; }),
      mc_rtc::gui::NumberInput("Max hand step [m]", [this]() { return maxStep_; },
                               [this](double s) { maxStep_ = std::max(0.05, s); }),
      mc_rtc::gui::NumberInput("Input delay [s]", [this]() { return inputDelay_; },
                               [this](double d) { inputDelay_ = std::max(0.0, d); }));

  ctl.gui()->addElement(
      this, {"Avatar", "Experiment Reset"},
      mc_rtc::gui::Label("Vive control state", [this]() { return controlStateName(); }),
      mc_rtc::gui::Label("Reset target", []() { return std::string("controller start pose"); }),
      mc_rtc::gui::Label("Start pose captured", [this]() { return experimentStartPoseCaptured_; }),
      mc_rtc::gui::Label("Reset status", [this]() -> const std::string & { return experimentResetStatus_; }),
      mc_rtc::gui::Label("Reset progress", [this]() { return experimentResetProgress_; }),
      mc_rtc::gui::Button("Reset to experiment start pose", [this]() { experimentResetRequested_ = true; }),
      mc_rtc::gui::Button("Abort reset", [this]() { experimentResetAbortRequested_ = true; }));

  // 实验日志记录实际运行参数，便于区分无延迟和有延迟试次。
  const std::string experimentLogPrefix = name() + "_experiment_";
  ctl.logger().addLogEntry(
      experimentLogPrefix + "input-delay-s", this, [this]() { return inputDelay_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "tracking-started", this, [this]() { return trackingStarted_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "condition-received", this, [this]() { return experimentConditionReceived_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "condition", this, [this]() { return experimentCondition_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "condition-name", this, [this]() -> const std::string & {
        return experimentConditionName_;
      });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "session-id", this, [this]() -> const std::string & {
        return experimentSessionId_;
      });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "assistance-enabled", this, [this]() { return experimentAssistanceEnabled_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "linear-velocity-feedforward", this,
      [this]() { return useLinearVelocityFeedforward_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "angular-velocity-feedforward", this,
      [this]() { return useAngularVelocityFeedforward_; });
  ctl.logger().addLogEntry(
      experimentLogPrefix + "linear-velocity-gain", this, [this]() { return linearVelocityGain_; });

  setupVelocityValidation(ctl);

  mc_rtc::log::success("[{}] Subscribed hands: {} ({}), {} ({})", name(), left_.topic, left_.serial, right_.topic,
                       right_.serial);
  mc_rtc::log::success("[{}] Subscribed trackers: left arm {} ({}), right arm {} ({}), waist {} ({})", name(),
                       leftArm_.topic, leftArm_.serial, rightArm_.topic, rightArm_.serial, waist_.topic, waist_.serial);
}

bool ViveHandBridge::run(mc_control::fsm::Controller & ctl)
{
  rclcpp::spin_some(node_);
  applyPendingExperimentCommand(ctl);
  updateDelayedInputs();

  const bool leftReset = resetPressed(left_);
  const bool rightReset = resetPressed(right_);
  // ROS may deliver both the press and release during one spin_some(). Consume the
  // callback-level latch so that a short menu press cannot disappear between cycles.
  const bool leftResetRising = left_.resetPressPending || (leftReset && !left_.previousResetButton);
  const bool rightResetRising = right_.resetPressPending || (rightReset && !right_.previousResetButton);
  left_.resetPressPending = false;
  right_.resetPressPending = false;
  if(!leftReset && !rightReset)
  {
    menuActionArmed_ = true;
    bothMenuHoldStartTime_ = -1.0;
    bothMenuHoldProgress_ = 0.0;
  }

  processExperimentReset(ctl);
  if(experimentResetActive_)
  {
    left_.previousResetButton = leftReset;
    right_.previousResetButton = rightReset;
    output("OK");
    return true;
  }

  const bool simultaneousPress =
      left_.resetPressTime >= 0.0 && right_.resetPressTime >= 0.0
      && std::abs(left_.resetPressTime - right_.resetPressTime) <= twoHandPressWindow_;
  const bool startChord = menuActionArmed_ && simultaneousPress && (leftResetRising || rightResetRising);

  if(trackingStarted_)
  {
    if(menuActionArmed_ && leftReset && rightReset)
    {
      if(bothMenuHoldStartTime_ < 0.0)
      {
        bothMenuHoldStartTime_ = now();
      }
      bothMenuHoldProgress_ = std::max(0.0, now() - bothMenuHoldStartTime_);
      if(bothMenuHoldProgress_ >= twoHandPauseHoldDuration_)
      {
        pauseTracking();
        menuActionArmed_ = false;
        ++pauseCount_;
        mc_rtc::log::warning("[{}] Vive control paused by two-hand menu hold", name());
      }
    }
    else
    {
      bothMenuHoldStartTime_ = -1.0;
      bothMenuHoldProgress_ = 0.0;
    }
  }
  else if((!requireInitialTwoHandReset_ && !trackingEverStarted_) || startChord)
  {
    if(startOrResumeTracking(ctl))
    {
      initialLeftResetSeen_ = true;
      initialRightResetSeen_ = true;
      menuActionArmed_ = false;
      ++startResumeCount_;
    }
  }

  left_.previousResetButton = leftReset;
  right_.previousResetButton = rightReset;

  if(!trackingStarted_)
  {
    holdCurrentTargets();
    output("OK");
    return true;
  }

  const bool leftGripper = gripperPressed(left_);
  const bool rightGripper = gripperPressed(right_);
  if(!leftReset && leftGripper && !left_.previousGripperButton)
  {
    left_.gripperCommandActive = true;
    left_.gripperClosed = !left_.gripperClosed;
    mc_rtc::log::info("[{}] Left gripper {}", name(), left_.gripperClosed ? "closed" : "open");
    publishGraspEvent("left", left_.gripperClosed ? "gripper_closed" : "gripper_opened", left_.latest.trigger);
  }
  if(!rightReset && rightGripper && !right_.previousGripperButton)
  {
    right_.gripperCommandActive = true;
    right_.gripperClosed = !right_.gripperClosed;
    mc_rtc::log::info("[{}] Right gripper {}", name(), right_.gripperClosed ? "closed" : "open");
    publishGraspEvent("right", right_.gripperClosed ? "gripper_closed" : "gripper_opened", right_.latest.trigger);
  }
  // 复位/启动键按住期间将夹具输入视为已经消费，松开时不会产生伪上升沿。
  left_.previousGripperButton = leftReset ? true : leftGripper;
  right_.previousGripperButton = rightReset ? true : rightGripper;

  const bool leftGraspNotification =
      left_.hasData && left_.latest.trigger >= graspNotificationTriggerThreshold_;
  const bool rightGraspNotification =
      right_.hasData && right_.latest.trigger >= graspNotificationTriggerThreshold_;
  if(leftGraspNotification && !left_.previousGraspNotificationButton)
  {
    publishGraspEvent("left", "grasp_notify", left_.latest.trigger);
  }
  if(rightGraspNotification && !right_.previousGraspNotificationButton)
  {
    publishGraspEvent("right", "grasp_notify", right_.latest.trigger);
  }
  left_.previousGraspNotificationButton = leftGraspNotification;
  right_.previousGraspNotificationButton = rightGraspNotification;

  updateHand(ctl, left_);
  updateHand(ctl, right_);
  updateArm(ctl, leftArm_);
  updateArm(ctl, rightArm_);
  updateGrippers(ctl);
  publishAdhesionCommand();

  output("OK");
  return true;
}

void ViveHandBridge::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().remove(name() + "::tracking_started");
  if(ctl.datastore().has("ANA::Vive::tracking_started"))
  {
    ctl.datastore().remove("ANA::Vive::tracking_started");
  }
  if(experimentConditionEnabled_)
  {
    const std::vector<std::string> keys = {experimentConditionKey_, experimentConditionNameKey_, experimentSessionKey_,
                                           experimentAssistanceKey_, experimentDelayKey_, experimentConditionReadyKey_};
    for(const auto & key : keys)
    {
      if(ctl.datastore().has(key))
      {
        ctl.datastore().remove(key);
      }
    }
  }
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
  for(const auto & entry : velocityValidationLogEntries_)
  {
    ctl.logger().removeLogEntry(entry);
  }
  velocityValidationLogEntries_.clear();
  left_.sub.reset();
  right_.sub.reset();
  leftArm_.sub.reset();
  rightArm_.sub.reset();
  waist_.sub.reset();
  controllerSubs_.clear();
  trackerSubs_.clear();
  adhesionPub_.reset();
  graspEventPub_.reset();
  experimentCommandSub_.reset();
  node_.reset();
  if(ownsRclcpp_)
  {
    rclcpp::shutdown();
  }
}

void ViveHandBridge::onController(HandState & hand, const Controller::SharedPtr msg)
{
  if(!hand.serial.empty() && msg->serial_number != hand.serial)
  {
    return;
  }
  pushSample(hand, *msg);
}

void ViveHandBridge::onAnyController(const std::string & topic, const Controller::SharedPtr msg)
{
  if(!msg->pose_is_valid)
  {
    return;
  }
  if(!left_.serial.empty() && msg->serial_number == left_.serial)
  {
    onController(left_, msg);
    return;
  }
  if(!right_.serial.empty() && msg->serial_number == right_.serial)
  {
    onController(right_, msg);
    return;
  }
  if(topic == left_.topic)
  {
    onController(left_, msg);
    return;
  }
  if(topic == right_.topic)
  {
    onController(right_, msg);
  }
}

void ViveHandBridge::onTracker(TrackerState & tracker, const Tracker::SharedPtr msg)
{
  if(!tracker.serial.empty() && msg->serial_number != tracker.serial)
  {
    return;
  }
  if(!msg->pose_is_valid)
  {
    return;
  }
  pushSample(tracker, *msg);
}

void ViveHandBridge::onAnyTracker(const std::string & topic, const Tracker::SharedPtr msg)
{
  if(!msg->pose_is_valid)
  {
    return;
  }
  if(!waist_.serial.empty() && msg->serial_number == waist_.serial)
  {
    onTracker(waist_, msg);
    return;
  }
  if(!leftArm_.serial.empty() && msg->serial_number == leftArm_.serial)
  {
    onTracker(leftArm_, msg);
    return;
  }
  if(!rightArm_.serial.empty() && msg->serial_number == rightArm_.serial)
  {
    onTracker(rightArm_, msg);
    return;
  }
  if(topic == waist_.topic)
  {
    onTracker(waist_, msg);
    return;
  }
  if(topic == leftArm_.topic)
  {
    onTracker(leftArm_, msg);
    return;
  }
  if(topic == rightArm_.topic)
  {
    onTracker(rightArm_, msg);
  }
}

void ViveHandBridge::onExperimentCommand(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & command = msg->data;
  const auto separator = command.find(':');
  const std::string conditionText = command.substr(0, separator);

  int condition = 0;
  try
  {
    condition = std::stoi(conditionText);
  }
  catch(const std::exception &)
  {
    mc_rtc::log::warning("[{}] Ignoring invalid experiment command '{}'", name(), command);
    return;
  }

  if(condition < 1 || condition > 4)
  {
    mc_rtc::log::warning("[{}] Experiment condition must be 1, 2, 3 or 4; received '{}'", name(), command);
    return;
  }

  const std::string session = separator == std::string::npos || separator + 1 >= command.size()
                                  ? "condition-" + std::to_string(condition)
                                  : command.substr(separator + 1);
  if(experimentConditionReceived_ && condition == experimentCondition_ && session == experimentSessionId_)
  {
    return;
  }
  if(pendingExperimentCommand_ && condition == pendingExperimentCondition_ && session == pendingExperimentSessionId_)
  {
    return;
  }

  pendingExperimentCondition_ = condition;
  pendingExperimentSessionId_ = session;
  pendingExperimentCommand_ = true;
}

void ViveHandBridge::applyPendingExperimentCommand(mc_control::fsm::Controller & ctl)
{
  if(!experimentConditionEnabled_ || !pendingExperimentCommand_)
  {
    return;
  }

  const bool replacingActiveSession = experimentConditionReceived_ && trackingStarted_;
  if(experimentRearmOnCommand_)
  {
    holdAndRearmTracking();
    if(replacingActiveSession)
    {
      mc_rtc::log::warning("[{}] New experiment session received; holding the current targets until both menu "
                           "buttons are confirmed again",
                           name());
    }
  }
  else
  {
    clearInputHistories();
  }

  experimentCondition_ = pendingExperimentCondition_;
  experimentSessionId_ = pendingExperimentSessionId_;
  experimentConditionName_ = experimentConditionName(experimentCondition_);
  experimentAssistanceEnabled_ = experimentCondition_ == 2 || experimentCondition_ == 4;
  inputDelay_ = experimentCondition_ >= 3 ? experimentDelayedInputSeconds_ : 0.0;
  experimentConditionReceived_ = true;
  pendingExperimentCommand_ = false;

  ctl.datastore().get<int>(experimentConditionKey_) = experimentCondition_;
  ctl.datastore().get<std::string>(experimentConditionNameKey_) = experimentConditionName_;
  ctl.datastore().get<std::string>(experimentSessionKey_) = experimentSessionId_;
  ctl.datastore().get<bool>(experimentAssistanceKey_) = experimentAssistanceEnabled_;
  ctl.datastore().get<double>(experimentDelayKey_) = inputDelay_;
  ctl.datastore().get<bool>(experimentConditionReadyKey_) = true;

  mc_rtc::log::success("[{}] Experiment session {}: condition {} ({}), input delay={:.1f}s, assistance={}",
                       name(),
                       experimentSessionId_,
                       experimentCondition_,
                       experimentConditionName_,
                       inputDelay_,
                       experimentAssistanceEnabled_);
}

void ViveHandBridge::holdAndRearmTracking()
{
  pauseTracking();
  initialLeftResetSeen_ = false;
  initialRightResetSeen_ = false;
  initialWaitingForDataLogged_ = false;
  menuActionArmed_ = false;
  clearInputHistories();
}

void ViveHandBridge::holdCurrentTargets()
{
  auto holdTarget = [](auto & state) {
    if(state.task)
    {
      state.task->target(state.zeroRobot);
      state.task->targetVel(sva::MotionVecd::Zero());
    }
    state.previousTargetValid = false;
    state.heldTargetVelocity = sva::MotionVecd::Zero();
  };
  holdTarget(left_);
  holdTarget(right_);
  holdTarget(leftArm_);
  holdTarget(rightArm_);
  if(headTask_)
  {
    headTask_->targetVel(sva::MotionVecd::Zero());
  }
}

bool ViveHandBridge::startOrResumeTracking(mc_control::fsm::Controller & ctl)
{
  const bool experimentReady = !experimentConditionRequired_ || experimentConditionReceived_;
  const bool waistReady = !useWaistReference_ || waist_.hasData;
  const bool armsReady = !initialZeroRequiresArmTrackers_ || (leftArm_.hasData && rightArm_.hasData);
  const bool handsReady = left_.hasData && right_.hasData && left_.task && right_.task;
  if(!experimentReady || !waistReady || !armsReady || !handsReady)
  {
    if(!initialWaitingForDataLogged_)
    {
      initialWaitingForDataLogged_ = true;
      mc_rtc::log::warning("[{}] Cannot start/resume Vive control: condition={}, hands={}, waist={}, arms={}",
                           name(), experimentReady, handsReady, waistReady, armsReady);
    }
    return false;
  }

  const bool firstStart = !trackingEverStarted_;
  if(!directWaistControl_ && firstStart)
  {
    setZero(ctl);
    if(!left_.zeroed || !right_.zeroed)
    {
      return false;
    }
  }
  if(directWaistControl_ && firstStart)
  {
    const std::string headZeroRequestKey = "ANA::ViveHeadBridge::zero_request";
    if(ctl.datastore().has(headZeroRequestKey))
    {
      // Only the HMD needs an initial relative heading. Hand/arm control remains
      // direct waist-relative and never establishes a button-triggered hand zero.
      ctl.datastore().get<bool>(headZeroRequestKey) = true;
    }
  }

  trackingStarted_ = true;
  trackingPaused_ = false;
  trackingEverStarted_ = true;
  trackingHandoverStartTime_ = -1.0;
  autoZeroDone_ = true;
  zeroRequested_ = false;
  initialWaitingForDataLogged_ = false;
  left_.previousTargetValid = false;
  right_.previousTargetValid = false;
  leftArm_.previousTargetValid = false;
  rightArm_.previousTargetValid = false;
  left_.heldTargetVelocity = sva::MotionVecd::Zero();
  right_.heldTargetVelocity = sva::MotionVecd::Zero();
  leftArm_.heldTargetVelocity = sva::MotionVecd::Zero();
  rightArm_.heldTargetVelocity = sva::MotionVecd::Zero();
  left_.previousGripperButton = true;
  right_.previousGripperButton = true;
  left_.previousGraspNotificationButton =
      left_.hasData && left_.latest.trigger >= graspNotificationTriggerThreshold_;
  right_.previousGraspNotificationButton =
      right_.hasData && right_.latest.trigger >= graspNotificationTriggerThreshold_;
  mc_rtc::log::success("[{}] Vive control {}", name(), firstStart ? "started" : "resumed");
  return true;
}

void ViveHandBridge::pauseTracking()
{
  auto captureAndHold = [](auto & state) {
    if(state.task)
    {
      state.zeroRobot = state.task->target();
      state.task->targetVel(sva::MotionVecd::Zero());
    }
    state.previousTargetValid = false;
    state.heldTargetVelocity = sva::MotionVecd::Zero();
  };
  captureAndHold(left_);
  captureAndHold(right_);
  captureAndHold(leftArm_);
  captureAndHold(rightArm_);
  if(headTask_)
  {
    headTask_->targetVel(sva::MotionVecd::Zero());
  }
  trackingStarted_ = false;
  trackingPaused_ = trackingEverStarted_;
  trackingHandoverStartTime_ = -1.0;
  bothMenuHoldStartTime_ = -1.0;
  bothMenuHoldProgress_ = 0.0;
}

void ViveHandBridge::captureExperimentStartPose(mc_control::fsm::Controller & ctl)
{
  if(!left_.task || !right_.task || !leftArm_.task || !rightArm_.task)
  {
    experimentResetStatus_ = "start pose unavailable: missing retargetting task";
    return;
  }
  experimentStartLeftHand_ = left_.task->target();
  experimentStartRightHand_ = right_.task->target();
  experimentStartLeftArm_ = leftArm_.task->target();
  experimentStartRightArm_ = rightArm_.task->target();
  if(headTask_)
  {
    experimentStartHead_ = headTask_->target();
  }
  auto posture = ctl.getPostureTask(ctl.robot().name());
  experimentStartPosture_ = posture->posture();
  const auto & currentQ = ctl.robot().mbc().q;
  // Keep the posture task's floating-base entry untouched. Record measured joint
  // coordinates so the reset target is the actual safe startup posture.
  for(size_t joint = 1; joint < experimentStartPosture_.size() && joint < currentQ.size(); ++joint)
  {
    if(experimentStartPosture_[joint].size() == currentQ[joint].size())
    {
      experimentStartPosture_[joint] = currentQ[joint];
    }
  }
  experimentStartPoseCaptured_ = true;
  experimentResetStatus_ = "ready";
  mc_rtc::log::success("[{}] Captured controller start pose for between-trial reset", name());
}

void ViveHandBridge::processExperimentReset(mc_control::fsm::Controller & ctl)
{
  if(experimentResetAbortRequested_)
  {
    experimentResetAbortRequested_ = false;
    experimentResetRequested_ = false;
    if(experimentResetActive_)
    {
      experimentResetActive_ = false;
      pauseTracking();
      experimentResetStatus_ = "aborted; holding current pose";
      mc_rtc::log::warning("[{}] Experiment pose reset aborted", name());
    }
  }

  if(experimentResetRequested_)
  {
    experimentResetRequested_ = false;
    if(trackingStarted_)
    {
      experimentResetStatus_ = "rejected: pause Vive control first";
      mc_rtc::log::warning("[{}] Reset rejected while Vive control is running", name());
    }
    else if(!experimentStartPoseCaptured_)
    {
      experimentResetStatus_ = "rejected: start pose is unavailable";
    }
    else if(left_.task && right_.task && leftArm_.task && rightArm_.task)
    {
      experimentResetFromLeftHand_ = left_.task->target();
      experimentResetFromRightHand_ = right_.task->target();
      experimentResetFromLeftArm_ = leftArm_.task->target();
      experimentResetFromRightArm_ = rightArm_.task->target();
      if(headTask_)
      {
        experimentResetFromHead_ = headTask_->target();
      }
      auto posture = ctl.getPostureTask(ctl.robot().name());
      experimentResetFromPosture_ = posture->posture();
      const auto & currentQ = ctl.robot().mbc().q;
      for(size_t joint = 1; joint < experimentResetFromPosture_.size() && joint < currentQ.size(); ++joint)
      {
        if(experimentResetFromPosture_[joint].size() == currentQ[joint].size())
        {
          experimentResetFromPosture_[joint] = currentQ[joint];
        }
      }
      experimentResetStartTime_ = now();
      experimentResetProgress_ = 0.0;
      experimentResetActive_ = true;
      trackingPaused_ = true;
      menuActionArmed_ = false;
      experimentResetStatus_ = "resetting";
      mc_rtc::log::warning("[{}] Returning robot to controller start pose over {:.1f}s", name(),
                           experimentResetDuration_);
    }
  }

  if(!experimentResetActive_)
  {
    return;
  }

  const double rawRatio = (now() - experimentResetStartTime_) / experimentResetDuration_;
  const double ratio = std::clamp(rawRatio, 0.0, 1.0);
  const double smoothRatio = ratio * ratio * (3.0 - 2.0 * ratio);
  experimentResetProgress_ = ratio;

  left_.zeroRobot = sva::interpolate(experimentResetFromLeftHand_, experimentStartLeftHand_, smoothRatio);
  right_.zeroRobot = sva::interpolate(experimentResetFromRightHand_, experimentStartRightHand_, smoothRatio);
  leftArm_.zeroRobot = sva::interpolate(experimentResetFromLeftArm_, experimentStartLeftArm_, smoothRatio);
  rightArm_.zeroRobot = sva::interpolate(experimentResetFromRightArm_, experimentStartRightArm_, smoothRatio);
  if(headTask_)
  {
    headTask_->target(sva::interpolate(experimentResetFromHead_, experimentStartHead_, smoothRatio));
    headTask_->targetVel(sva::MotionVecd::Zero());
  }
  holdCurrentTargets();

  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(posture && experimentResetFromPosture_.size() == experimentStartPosture_.size())
  {
    auto target = experimentResetFromPosture_;
    for(size_t joint = 0; joint < target.size(); ++joint)
    {
      if(target[joint].size() != experimentStartPosture_[joint].size())
      {
        continue;
      }
      for(size_t dof = 0; dof < target[joint].size(); ++dof)
      {
        target[joint][dof] = (1.0 - smoothRatio) * experimentResetFromPosture_[joint][dof]
                             + smoothRatio * experimentStartPosture_[joint][dof];
      }
    }
    posture->posture(target);
  }

  if(ratio >= 1.0)
  {
    experimentResetActive_ = false;
    experimentResetStatus_ = "complete; waiting for two-hand menu press";
    experimentResetProgress_ = 1.0;
    mc_rtc::log::success("[{}] Experiment pose reset complete", name());
  }
}

std::string ViveHandBridge::controlStateName() const
{
  if(experimentResetActive_)
  {
    return "RESETTING";
  }
  if(trackingStarted_)
  {
    return "RUNNING";
  }
  if(trackingPaused_)
  {
    return "PAUSED";
  }
  return "READY";
}

void ViveHandBridge::clearInputHistories()
{
  auto clearTracker = [](auto & state) {
    state.history.clear();
    state.hasData = false;
    state.latestSampleTime = 0.0;
    state.previousTargetValid = false;
    state.heldTargetVelocity = sva::MotionVecd::Zero();
  };
  clearTracker(left_);
  clearTracker(right_);
  clearTracker(leftArm_);
  clearTracker(rightArm_);
  clearTracker(waist_);

  left_.rawResetButton = false;
  right_.rawResetButton = false;
  left_.previousReceivedResetButton = false;
  right_.previousReceivedResetButton = false;
  left_.resetPressPending = false;
  right_.resetPressPending = false;
  left_.resetPressTime = -1.0;
  right_.resetPressTime = -1.0;
  left_.previousResetButton = false;
  right_.previousResetButton = false;
  // A grip/trigger already held while a new condition is selected must not become a fresh command.
  left_.previousGripperButton = true;
  right_.previousGripperButton = true;
  left_.previousGraspNotificationButton = true;
  right_.previousGraspNotificationButton = true;
}

std::string ViveHandBridge::experimentConditionName(int condition) const
{
  switch(condition)
  {
    case 1:
      return "no-delay / no-assistance";
    case 2:
      return "no-delay / assistance";
    case 3:
      return "1.5s-delay / no-assistance";
    case 4:
      return "1.5s-delay / assistance";
    default:
      return "invalid";
  }
}

void ViveHandBridge::updateDelayedInputs()
{
  // inputDelay_ 为 0 时使用最新样本；大于 0 时从历史队列中取延迟后的样本。
  selectDelayedSample(left_);
  selectDelayedSample(right_);
  selectDelayedSample(waist_);
  selectDelayedSample(leftArm_);
  selectDelayedSample(rightArm_);
}

double ViveHandBridge::now() const
{
  if(node_)
  {
    return node_->now().seconds();
  }
  return 0.0;
}

void ViveHandBridge::pushSample(HandState & state, const Controller & msg)
{
  const double sampleTime = now();
  state.rawResetButton = msg.menu_button;
  if(msg.menu_button && !state.previousReceivedResetButton)
  {
    state.resetPressPending = true;
    state.resetPressTime = sampleTime;
    ++state.resetPressCount;
  }
  state.previousReceivedResetButton = msg.menu_button;
  state.history.push_back({sampleTime, msg});
  if(inputDelay_ <= 0.0)
  {
    state.latest = msg;
    state.latestSampleTime = sampleTime;
    state.hasData = msg.pose_is_valid;
  }
}

void ViveHandBridge::pushSample(TrackerState & state, const Tracker & msg)
{
  const double sampleTime = now();
  state.history.push_back({sampleTime, msg});
  if(inputDelay_ <= 0.0)
  {
    state.latest = msg;
    state.latestSampleTime = sampleTime;
    state.hasData = msg.pose_is_valid;
  }
}

template<typename StateT>
void ViveHandBridge::selectDelayedSample(StateT & state)
{
  if(state.history.empty())
  {
    return;
  }

  if(inputDelay_ <= 0.0)
  {
    // 无延迟模式只保留最新帧，避免历史队列无限增长。
    while(state.history.size() > 1)
    {
      state.history.pop_front();
    }
    return;
  }

  const double targetTime = now() - inputDelay_;
  while(state.history.size() > 1 && state.history[1].time <= targetTime)
  {
    state.history.pop_front();
  }

  if(state.history.front().time <= targetTime)
  {
    state.latest = state.history.front().msg;
    state.latestSampleTime = state.history.front().time;
    state.hasData = state.latest.pose_is_valid;
  }
  else
  {
    state.hasData = false;
  }

  const double keepWindow = inputDelay_ + 2.0;
  const double oldestUseful = now() - keepWindow;
  while(state.history.size() > 1 && state.history.front().time < oldestUseful)
  {
    state.history.pop_front();
  }
}

Eigen::Vector3d ViveHandBridge::vivePosition(const Controller & msg) const
{
  return {static_cast<double>(msg.pose_matrix[3]), static_cast<double>(msg.pose_matrix[7]),
          static_cast<double>(msg.pose_matrix[11])};
}

Eigen::Matrix3d ViveHandBridge::viveRotation(const Controller & msg) const
{
  Eigen::Matrix3d r;
  r << static_cast<double>(msg.pose_matrix[0]), static_cast<double>(msg.pose_matrix[1]),
      static_cast<double>(msg.pose_matrix[2]), static_cast<double>(msg.pose_matrix[4]),
      static_cast<double>(msg.pose_matrix[5]), static_cast<double>(msg.pose_matrix[6]),
      static_cast<double>(msg.pose_matrix[8]), static_cast<double>(msg.pose_matrix[9]),
      static_cast<double>(msg.pose_matrix[10]);
  return r;
}

Eigen::Vector3d ViveHandBridge::controllerHandOffset(const HandState & hand) const
{
  return (&hand == &left_) ? leftControllerHandOffset_ : rightControllerHandOffset_;
}

Eigen::Vector3d ViveHandBridge::trackerPosition(const Tracker & msg) const
{
  return {static_cast<double>(msg.pose_matrix[3]), static_cast<double>(msg.pose_matrix[7]),
          static_cast<double>(msg.pose_matrix[11])};
}

Eigen::Matrix3d ViveHandBridge::trackerRotation(const Tracker & msg) const
{
  Eigen::Matrix3d r;
  r << static_cast<double>(msg.pose_matrix[0]), static_cast<double>(msg.pose_matrix[1]),
      static_cast<double>(msg.pose_matrix[2]), static_cast<double>(msg.pose_matrix[4]),
      static_cast<double>(msg.pose_matrix[5]), static_cast<double>(msg.pose_matrix[6]),
      static_cast<double>(msg.pose_matrix[8]), static_cast<double>(msg.pose_matrix[9]),
      static_cast<double>(msg.pose_matrix[10]);
  return r;
}

Eigen::Vector3d ViveHandBridge::relativeHandPosition(const HandState & hand) const
{
  const Eigen::Matrix3d handRotation = viveRotation(hand.latest);
  // OpenVR 给出的 controller 位置是手柄内部追踪原点，不一定等于操作者手掌/手腕参考点。
  // 这里用手柄本地坐标下的固定偏移，把控制点从手柄原点移到更接近人手的位置；
  // 这样纯旋转手腕时，目标球不会因为手柄原点偏移而画出很大的弧线。
  // 自然站立校准使用录制时的 controller 追踪原点，先不叠加旧实验得到的手掌 offset。
  // 这样位置方向校准和运行时使用的是同一个几何点，避免旧 offset 再次耦合平移与旋转。
  const Eigen::Vector3d offset = neutralBodyCalibration_ ? Eigen::Vector3d::Zero() : controllerHandOffset(hand);
  const Eigen::Vector3d handPosition = vivePosition(hand.latest) + handRotation * offset;
  if(useWaistReference_ && waist_.hasData)
  {
    // 有腰部 tracker 时，把手柄位置转到腰部局部坐标，减小 base station 摆放变化的影响。
    return trackerRotation(waist_.latest).transpose() * (handPosition - trackerPosition(waist_.latest));
  }
  return handPosition;
}

Eigen::Matrix3d ViveHandBridge::relativeHandRotation(const HandState & hand) const
{
  const Eigen::Matrix3d handRotation = viveRotation(hand.latest);
  if(useWaistReference_ && waist_.hasData)
  {
    // 姿态也转到腰部局部坐标；这样操作者/基站整体转向时，手腕相对身体的方向仍然一致。
    return trackerRotation(waist_.latest).transpose() * handRotation;
  }
  return handRotation;
}

Eigen::Matrix3d ViveHandBridge::relativeTrackerRotation(const TrackerState & tracker) const
{
  const Eigen::Matrix3d rotation = trackerRotation(tracker.latest);
  if(useWaistReference_ && waist_.hasData)
  {
    // 肘部 tracker 的姿态同样使用腰部局部坐标，避免直接依赖 SteamVR 世界坐标。
    return trackerRotation(waist_.latest).transpose() * rotation;
  }
  return rotation;
}

Eigen::Vector3d ViveHandBridge::mapDeltaToRobot(const Eigen::Vector3d & delta, bool leftHand, bool applyMaxStep) const
{
  Eigen::Vector3d out;
  // Vive 的前后/左右/上下轴与 HRP4CR 机器人坐标不同，这里做固定轴映射。
  out << delta.z(), -delta.y(), delta.x();
  if(leftHand && leftPositionHorizontalFix_)
  {
    // 现场调试用的左手水平面修正。不同 mode 只改变水平面 x/y，
    // 上下 z 保持不变；这样可以只改 yaml 重启程序，不需要每次重新编译。
    const double x = out.x();
    const double y = out.y();
    switch(leftPositionHorizontalMode_)
    {
      case 1:
        out.x() = y;
        out.y() = -x;
        break;
      case 2:
        out.x() = -y;
        out.y() = x;
        break;
      case 3:
        out.x() = y;
        out.y() = x;
        break;
      case 4:
        out.x() = -y;
        out.y() = -x;
        break;
      case 5:
        out.y() = -y;
        break;
      case 6:
        out.x() = -x;
        break;
      case 7:
        out.x() = -x;
        out.y() = -y;
        break;
      default:
        break;
    }
  }
  out = scale_ * axisScale_.cwiseProduct(out);
  if(applyMaxStep && out.norm() > maxStep_)
  {
    out = out.normalized() * maxStep_;
  }
  return out;
}

Eigen::Vector3d ViveHandBridge::mapWaistRelativePositionToRobot(const Eigen::Vector3d & position, bool leftHand) const
{
  // direct waist 模式使用离线录制动作拟合出的固定坐标变换：
  // 输入是 Vive waist tracker 局部坐标下的手部位置，输出是机器人 BODY 坐标下的 [前后, 左右, 上下]。
  Eigen::Vector3d out = (neutralBodyCalibration_ ? neutralBodyFromWaist_ : waistToRobotPosition_) * position;
  if(!neutralBodyCalibration_ && directWaistKeepHandSide_)
  {
    // 只修正左右手落在身体中线反侧的问题。
    // 这里不翻转“左右移动趋势”，而是保持左手目标在 BODY 左侧、右手目标在 BODY 右侧。
    out.y() = leftHand ? std::abs(out.y()) : -std::abs(out.y());
  }
  if(!neutralBodyCalibration_ && leftHand && leftPositionHorizontalFix_)
  {
    const double x = out.x();
    const double y = out.y();
    switch(leftPositionHorizontalMode_)
    {
      case 1:
        out.x() = y;
        out.y() = -x;
        break;
      case 2:
        out.x() = -y;
        out.y() = x;
        break;
      case 3:
        out.x() = y;
        out.y() = x;
        break;
      case 4:
        out.x() = -y;
        out.y() = -x;
        break;
      case 5:
        out.y() = -y;
        break;
      case 6:
        out.x() = -x;
        break;
      case 7:
        out.x() = -x;
        out.y() = -y;
        break;
      default:
        break;
    }
  }
  return scale_ * axisScale_.cwiseProduct(out);
}

Eigen::Matrix3d ViveHandBridge::rpyDegreesToMatrix(const Eigen::Vector3d & rpyDegrees) const
{
  const double degToRad = M_PI / 180.0;
  const Eigen::AngleAxisd roll(rpyDegrees.x() * degToRad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpyDegrees.y() * degToRad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpyDegrees.z() * degToRad, Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

sva::PTransformd ViveHandBridge::blendFromTrackingHold(const sva::PTransformd & held,
                                                       const sva::PTransformd & target) const
{
  if(trackingHandoverStartTime_ < 0.0 || trackingHandoverDuration_ <= 0.0)
  {
    return target;
  }
  const double rawRatio = (now() - trackingHandoverStartTime_) / trackingHandoverDuration_;
  const double ratio = std::clamp(rawRatio, 0.0, 1.0);
  // 三次平滑步进使接管开始和结束时速度都回到零，避免第一帧产生冲击。
  const double smoothRatio = ratio * ratio * (3.0 - 2.0 * ratio);
  const Eigen::Vector3d translation =
      (1.0 - smoothRatio) * held.translation() + smoothRatio * target.translation();
  const Eigen::Quaterniond heldRotation(held.rotation());
  const Eigen::Quaterniond targetRotation(target.rotation());
  const Eigen::Matrix3d rotation = heldRotation.slerp(smoothRatio, targetRotation).normalized().toRotationMatrix();
  return sva::PTransformd(rotation, translation);
}

Eigen::Vector3d ViveHandBridge::rotationVectorFromMatrix(const Eigen::Matrix3d & rotation) const
{
  Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis();
}

Eigen::Matrix3d ViveHandBridge::matrixFromRotationVector(const Eigen::Vector3d & rotationVector) const
{
  const double angle = rotationVector.norm();
  if(angle < 1e-12)
  {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

Eigen::Matrix3d ViveHandBridge::mapHandRotationDeltaToRobot(const Eigen::Matrix3d & delta,
                                                            const Eigen::Matrix3d & sideCorrection,
                                                            bool leftHand) const
{
  Eigen::Matrix3d viveToRobot;
  // 手柄姿态也需要用同一套坐标变换映射到机器人手部任务坐标。
  viveToRobot << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  // 在复位模式下，现场观察到两个手腕的上下翻转方向同时反了：
  // 操作者手心向上时机器人手心向下。这里允许把相对姿态增量取逆，
  // 保留成配置项，方便之后如果别的轴不合适可以一行切回。
  const Eigen::Matrix3d correctedDelta = invertHandOrientationDelta_ ? delta.transpose() : delta;
  Eigen::Matrix3d mapped = viveToRobot * correctedDelta * viveToRobot.transpose();

  Eigen::Matrix3d palmCorrection;
  // 根据现场观察，手掌法向的前后/上下轴差了约 90 度：
  // 手心向下会变成机器人手心向后，手心向上会变成机器人手心向前。
  // 这里用相对旋转的共轭校正轴定义，保持零点姿态本身不跳变。
  palmCorrection << 0, 0, -1, 0, 1, 0, 1, 0, 0;
  Eigen::Matrix3d result = palmCorrection * mapped * palmCorrection.transpose();
  const int handRotationAxisMode = leftHand ? leftHandRotationAxisMode_ : rightHandRotationAxisMode_;
  if(handRotationAxisMode != 0)
  {
    // 每只手只允许启用一个轴排列修正，避免多次现场补丁互相抵消。
    // 这里修正的是“相对旋转增量”的坐标轴，不改变复位时的默认手掌朝向。
    Eigen::Matrix3d axisFix = Eigen::Matrix3d::Identity();
    switch(handRotationAxisMode)
    {
      case 1:
        // 现场当前观测：pitch->yaw, yaw->roll, roll->pitch。
        // 反向排列：robot roll<-current yaw, pitch<-current roll, yaw<-current pitch。
        axisFix << 0, 1, 0, 0, 0, 1, 1, 0, 0;
        break;
      case 2:
        // 备选循环方向：robot roll<-current pitch, pitch<-current yaw, yaw<-current roll。
        axisFix << 0, 0, 1, 1, 0, 0, 0, 1, 0;
        break;
      case 3:
        // 只交换 roll/pitch，保留 yaw。
        // 现场确认轴已经对应，但 pitch/roll 方向相反；因此同时翻转这两个轴的方向。
        axisFix << 0, 1, 0, 1, 0, 0, 0, 0, 1;
        break;
      case 4:
        // 只交换 pitch/yaw，保留 roll。
        axisFix << 1, 0, 0, 0, 0, 1, 0, 1, 0;
        break;
      case 5:
        // 只交换 roll/yaw，保留 pitch。
        axisFix << 0, 0, 1, 0, 1, 0, 1, 0, 0;
        break;
      default:
        break;
    }
    result = axisFix * result * axisFix.transpose();
    if(leftHand && handRotationAxisMode == 3)
    {
      // 左手现场确认轴已正确，但 yaw/pitch 方向相反，roll 正常。
      // 因此只翻转 pitch/yaw，不改变 roll。
      const Eigen::Matrix3d yawPitchSignFix = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
      result = yawPitchSignFix * result * yawPitchSignFix.transpose();
    }
    if(!leftHand && handRotationAxisMode == 3)
    {
      const Eigen::Matrix3d pitchRollSignFix = Eigen::DiagonalMatrix<double, 3>(-1.0, -1.0, 1.0);
      result = pitchRollSignFix * result * pitchRollSignFix.transpose();
      // 现场确认 pitch 已经正确，yaw 和 roll 的方向仍然相反。
      // 这里只翻转 yaw/roll，保持 pitch 不变。
      const Eigen::Matrix3d yawRollSignFix = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, 1.0);
      result = yawRollSignFix * result * yawRollSignFix.transpose();
    }
  }
  // 左右手实际握持手柄时，手柄本体坐标到机器人手掌坐标还差一个固定偏置。
  // 这个偏置放在 YAML 里，便于现场根据“手心方向”继续微调。
  return sideCorrection * result;
}

Eigen::Matrix3d ViveHandBridge::mapArmRotationDeltaToRobot(const Eigen::Matrix3d & delta) const
{
  Eigen::Matrix3d viveToRobot;
  // 肘部 tracker 驱动的是大臂 retargeting task。
  // 复位模式下腋下开合方向应和人体一致；如果现场绑法导致趋势反了，
  // 可以在 YAML 里打开 invert_arm_orientation_delta。
  viveToRobot << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  const Eigen::Matrix3d correctedDelta = invertArmOrientationDelta_ ? delta.transpose() : delta;
  Eigen::Matrix3d mapped = viveToRobot * correctedDelta * viveToRobot.transpose();
  // arm tracker 的腋下开合和扩胸已经基本符合直觉，但现场观察到前举/后摆轴符号相反。
  // 这里转成旋转向量后只按配置翻转指定分量，避免整体取逆破坏已经正确的轴。
  Eigen::Vector3d rotvec = rotationVectorFromMatrix(mapped);
  rotvec = armRotationSignFix_.cwiseProduct(rotvec);
  return matrixFromRotationVector(rotvec);
}

void ViveHandBridge::setZero(mc_control::fsm::Controller &)
{
  if(useWaistReference_ && !waist_.hasData)
  {
    mc_rtc::log::warning("[{}] Cannot zero Vive input: waist tracker is required as reference but has no data", name());
    return;
  }

  auto zeroTracker = [this](TrackerState & tracker, const std::string & label, bool useWaistFrame) {
    if(!tracker.hasData)
    {
      mc_rtc::log::warning("[{}] Cannot zero {}, no tracker data", name(), label);
      return;
    }
    // 肘部/腰部 tracker 的零点用于后续姿态相对量，避免直接使用绝对 SteamVR 坐标。
    tracker.zeroPosition = trackerPosition(tracker.latest);
    tracker.zeroRotation = useWaistFrame ? relativeTrackerRotation(tracker) : trackerRotation(tracker.latest);
    if(tracker.task)
    {
      tracker.zeroRobot = tracker.task->target();
    }
    tracker.zeroed = true;
    mc_rtc::log::success("[{}] Zeroed {} tracker", name(), label);
  };
  zeroTracker(waist_, "waist", false);

  auto zeroHand = [this](HandState & hand, const std::string & label) {
    if(!hand.hasData || !hand.task)
    {
      mc_rtc::log::warning("[{}] Cannot zero {}, data {}, task {}", name(), label, hand.hasData,
                           static_cast<bool>(hand.task));
      return;
    }
    // 零点同时记录腰部坐标下的 Vive 位姿和当前机器人任务目标，之后所有控制都作为相对运动叠加。
    hand.zeroVive = relativeHandPosition(hand);
    hand.zeroRotation = relativeHandRotation(hand);
    hand.zeroRobot = hand.task->target();
    hand.zeroed = true;
    mc_rtc::log::success("[{}] Zeroed {} hand", name(), label);
  };
  zeroHand(left_, "left");
  zeroHand(right_, "right");
  zeroTracker(leftArm_, "left arm", true);
  zeroTracker(rightArm_, "right arm", true);
  left_.previousTargetValid = false;
  right_.previousTargetValid = false;
  leftArm_.previousTargetValid = false;
  rightArm_.previousTargetValid = false;
}

void ViveHandBridge::updateHand(mc_control::fsm::Controller & ctl, HandState & hand)
{
  if(!hand.hasData || !hand.task)
  {
    return;
  }
  sva::PTransformd target = directWaistControl_ ? hand.task->target() : hand.zeroRobot;
  if(directWaistControl_)
  {
    if(useWaistReference_ && !waist_.hasData)
    {
      return;
    }
    if(!ctl.robot().hasFrame(robotWaistFrame_))
    {
      mc_rtc::log::warning("[{}] Robot waist frame '{}' not found; cannot use direct waist control", name(),
                           robotWaistFrame_);
      return;
    }
    // direct 模式不取第一帧为零点：手柄相对腰部的位置本身就是控制输入。
    // 例如手柄在腰部左侧 10cm，机器人手部目标也会相对 BODY 往对应方向移动 10cm。
    const Eigen::Vector3d relativeToWaist = relativeHandPosition(hand);
    const bool isLeftHand = &hand == &left_;
    Eigen::Vector3d mappedRelative = mapWaistRelativePositionToRobot(relativeToWaist, isLeftHand);
    // waistReferenceOffsetBody_ 表示虚拟人体腰部中心相对实体后腰 tracker 的固定位置。
    // 手相对虚拟腰部的位置为 (手 - tracker) - (虚拟腰部 - tracker)，因此在 BODY 坐标中相减。
    // 该补偿统一作用于左右手，不改变方向矩阵、旋转映射或运动比例。
    mappedRelative -= waistReferenceOffsetBody_;
    if(neutralBodyCalibration_ && wristPivotCompensation_)
    {
      // OpenVR 的 position 位于手柄内部追踪原点；操作者只转手腕时，该原点会绕真实手腕画圆。
      // 离线拟合得到 r_device，使 p_controller + R_controller*r_device 近似固定手腕中心。
      // 这里减去自然下垂姿势下的常量偏移，只保留旋转产生的位置增量：
      //   delta_p = (R_body_hand - I) * r_hand
      // 因此不会改变已经确认的静态位置映射，只抵消翻手腕时的圆弧运动。
      const Eigen::Matrix3d & correction =
          isLeftHand ? leftHandNeutralCorrection_ : rightHandNeutralCorrection_;
      const Eigen::Vector3d & controllerOffset =
          isLeftHand ? leftControllerHandOffset_ : rightControllerHandOffset_;
      const Eigen::Matrix3d bodyDevice = neutralBodyFromWaist_ * relativeHandRotation(hand);
      const Eigen::Matrix3d bodyHand = bodyDevice * correction;
      const Eigen::Vector3d handFrameOffset = correction.transpose() * controllerOffset;
      mappedRelative += (bodyHand - Eigen::Matrix3d::Identity()) * handFrameOffset;
    }
    // BODY 坐标顺序为 [前, 左, 上]。正的 z 偏移等效于把实体 waist tracker 向下移动，
    // 但不会改变 waist tracker 的安装姿态，也不会影响 arm/head 的方向映射。
    mappedRelative += handPositionOffsetBody_;
    if(directWaistMaxDistance_ > 0.0 && mappedRelative.norm() > directWaistMaxDistance_)
    {
      mappedRelative = mappedRelative.normalized() * directWaistMaxDistance_;
    }
    target.translation() = ctl.robot().frame(robotWaistFrame_).position().translation() + mappedRelative;
    if(debugHandMapping_)
    {
      const double t = now();
      if(t - lastHandMappingDebugTime_ > debugHandMappingInterval_)
      {
        lastHandMappingDebugTime_ = t;
        mc_rtc::log::info("[{}] direct waist mapping: side={}, waist_pos=[{:.3f}, {:.3f}, {:.3f}], "
                          "robot_body_rel=[{:.3f}, {:.3f}, {:.3f}]",
                          name(),
                          isLeftHand ? "left" : "right",
                          relativeToWaist.x(),
                          relativeToWaist.y(),
                          relativeToWaist.z(),
                          mappedRelative.x(),
                          mappedRelative.y(),
                          mappedRelative.z());
      }
    }
  }
  else
  {
    if(!hand.zeroed)
    {
      return;
    }
    const Eigen::Vector3d deltaVive = relativeHandPosition(hand) - hand.zeroVive;
    // 传统复位模式：手部位置只应用相对零点的位移，并受 scale/axis_scale/max_step 限制。
    const bool isLeftHand = &hand == &left_;
    const Eigen::Vector3d mappedDelta = mapDeltaToRobot(deltaVive, isLeftHand);
    target.translation() = hand.zeroRobot.translation() + mappedDelta;
    if(debugHandMapping_)
    {
      const double t = now();
      if(t - lastHandMappingDebugTime_ > debugHandMappingInterval_)
      {
        lastHandMappingDebugTime_ = t;
        mc_rtc::log::info("[{}] hand mapping debug: side={}, vive_delta=[{:.3f}, {:.3f}, {:.3f}], "
                          "robot_delta=[{:.3f}, {:.3f}, {:.3f}], left_mode={}",
                          name(),
                          isLeftHand ? "left" : "right",
                          deltaVive.x(),
                          deltaVive.y(),
                          deltaVive.z(),
                          mappedDelta.x(),
                          mappedDelta.y(),
                          mappedDelta.z(),
                          leftPositionHorizontalMode_);
      }
    }
  }
  if(controlHandOrientation_)
  {
    if(directWaistControl_)
    {
      if(neutralBodyCalibration_)
      {
        // 新校准路径：先把设备姿态从 waist tracker 坐标转到人体坐标，
        // 再右乘设备安装修正。自然下垂时结果严格为红前、绿左、蓝上。
        // 这里不再经过旧的轴交换、符号翻转或按键零点逻辑。
        const bool isLeftHand = &hand == &left_;
        const Eigen::Matrix3d & correction =
            isLeftHand ? leftHandNeutralCorrection_ : rightHandNeutralCorrection_;
        const Eigen::Matrix3d bodyDevice = neutralBodyFromWaist_ * relativeHandRotation(hand);
        const Eigen::Matrix3d bodyHand = bodyDevice * correction;
        // OpenVR 的 pose_matrix 表示设备坐标轴在参考坐标中的主动旋转，
        // sva::PTransformd::rotation() 使用相反的坐标变换约定，因此写入任务目标前需要转置。
        // 自然下垂时 bodyHand=I，不受影响；运动后的旋转方向由此与人体保持一致。
        target.rotation() = bodyHand.transpose() * ctl.robot().frame(robotWaistFrame_).position().rotation();
      }
      else
      {
        // direct 模式下，手柄相对腰部的姿态直接映射到机器人身体坐标。
        const Eigen::Matrix3d & sideCorrection =
            (&hand == &left_) ? leftHandOrientationCorrection_ : rightHandOrientationCorrection_;
        // direct 模式下不再使用“按键零点”。位置由 waist-relative 绝对位置决定；
        // 姿态则先通过旧的手柄轴映射得到目标，再用一个固定轴修正把显示轴对齐成
        // 红=前、绿=左、蓝=上。这个修正只改变坐标轴方向，不改变手的位置。
        Eigen::Matrix3d directAxisFix = Eigen::Matrix3d::Identity();
        if(directHandRotationAxisMode_ == 1)
        {
          // direct 模式最终层轴修正：输入 yaw 表现成 roll、输入 roll 表现成 yaw 时，
          // 交换 roll/yaw，保持 pitch 不变。
          directAxisFix << 0, 0, 1, 0, 1, 0, 1, 0, 0;
        }
        const Eigen::Matrix3d directSignFix = directHandRotationSignFix_.asDiagonal();
        const Eigen::Matrix3d mappedRotation =
            mapHandRotationDeltaToRobot(relativeHandRotation(hand), sideCorrection, &hand == &left_);
        target.rotation() = directHandOrientationAxisFix_ * directSignFix * directAxisFix * mappedRotation
                            * directAxisFix.transpose() * directSignFix.transpose()
                            * ctl.robot().frame(robotWaistFrame_).position().rotation();
      }
    }
    else
    {
      // 传统复位模式下，姿态仍使用相对零点的旋转。
      const Eigen::Matrix3d deltaRotation = relativeHandRotation(hand) * hand.zeroRotation.transpose();
      const Eigen::Matrix3d & sideCorrection =
          (&hand == &left_) ? leftHandOrientationCorrection_ : rightHandOrientationCorrection_;
      target.rotation() = mapHandRotationDeltaToRobot(deltaRotation, sideCorrection, &hand == &left_) * hand.zeroRobot.rotation();
    }
  }
  if(directWaistControl_)
  {
    target = blendFromTrackingHold(hand.zeroRobot, target);
  }
  hand.task->target(target);
  const auto worldVelocity =
      computeTargetVelocity(hand.previousTarget,
                            hand.previousTargetTime,
                            hand.previousTargetValid,
                            hand.heldTargetVelocity,
                            target,
                            std::max(hand.latestSampleTime,
                                     useWaistReference_ ? waist_.latestSampleTime : 0.0),
                            velocityValidation_ ? &hand.velocityDiagnostics : nullptr);
  const auto commandWorldVelocity =
      useVelocityFeedforward_ ? feedforwardVelocity(worldVelocity) : sva::MotionVecd::Zero();
  if(velocityValidation_)
  {
    updateVelocityDiagnostics(hand, target, worldVelocity, commandWorldVelocity);
  }
  // TransformTask converts this world spatial velocity with the current controlled hand frame.
  // Keeping the derivative in world avoids orientation-dependent per-axis sign corrections.
  hand.task->targetVel(commandWorldVelocity);
  publishLegacyPoseVelocity(hand.legacyPosePub, hand.legacyVelocityPub, target, worldVelocity);
}

void ViveHandBridge::updateArm(mc_control::fsm::Controller & ctl, TrackerState & tracker)
{
  if(!tracker.hasData || !tracker.task)
  {
    return;
  }
  sva::PTransformd target = directWaistControl_ ? tracker.task->target() : tracker.zeroRobot;
  if(directWaistControl_)
  {
    if(useWaistReference_ && !waist_.hasData)
    {
      return;
    }
    if(!ctl.robot().hasFrame(robotWaistFrame_))
    {
      return;
    }
    if(neutralBodyCalibration_)
    {
      // arm 与 hand 使用同一个人体坐标基准，但各自保留独立的 tracker 安装修正矩阵。
      const bool isLeftArm = &tracker == &leftArm_;
      const Eigen::Matrix3d & correction = isLeftArm ? leftArmNeutralCorrection_ : rightArmNeutralCorrection_;
      const Eigen::Matrix3d bodyDevice = neutralBodyFromWaist_ * relativeTrackerRotation(tracker);
      const Eigen::Matrix3d bodyArm = bodyDevice * correction;
      target.rotation() = bodyArm.transpose() * ctl.robot().frame(robotWaistFrame_).position().rotation();
    }
    else
    {
      // 旧 direct 模式：保留之前逐轴调试得到的 arm 映射，供随时回退。
      target.rotation() = mapArmRotationDeltaToRobot(relativeTrackerRotation(tracker))
                          * ctl.robot().frame(robotWaistFrame_).position().rotation();
    }
  }
  else
  {
    if(!tracker.zeroed)
    {
      return;
    }
    // 传统复位模式：肘部 tracker 只应用相对零点的姿态变化。
    const Eigen::Matrix3d delta = relativeTrackerRotation(tracker) * tracker.zeroRotation.transpose();
    target.rotation() = mapArmRotationDeltaToRobot(delta) * tracker.zeroRobot.rotation();
  }
  if(directWaistControl_)
  {
    target = blendFromTrackingHold(tracker.zeroRobot, target);
  }
  tracker.task->target(target);
  const auto worldVelocity =
      computeTargetVelocity(tracker.previousTarget,
                            tracker.previousTargetTime,
                            tracker.previousTargetValid,
                            tracker.heldTargetVelocity,
                            target,
                            std::max(tracker.latestSampleTime,
                                     useWaistReference_ ? waist_.latestSampleTime : 0.0));
  auto commandWorldVelocity =
      useVelocityFeedforward_ ? feedforwardVelocity(worldVelocity) : sva::MotionVecd::Zero();
  if(useAngularVelocityFeedforward_)
  {
    // arm 姿态目标已经按 rotvec 做了单轴校正；这里单独修正角速度前馈。
    // 当前现场现象是腋下开合会出现“反向蓄力”，因此只先翻转对应的角速度轴。
    commandWorldVelocity.angular() = armAngularVelocitySignFix_.cwiseProduct(commandWorldVelocity.angular());
  }
  tracker.task->targetVel(commandWorldVelocity);
  publishLegacyPoseVelocity(tracker.legacyPosePub, tracker.legacyVelocityPub, target, worldVelocity);
}

sva::MotionVecd ViveHandBridge::computeTargetVelocity(sva::PTransformd & previousTarget,
                                                      double & previousTargetTime,
                                                      bool & previousTargetValid,
                                                      sva::MotionVecd & heldTargetVelocity,
                                                      const sva::PTransformd & target,
                                                      double sampleTime,
                                                      HandState::VelocityDiagnostics * diagnostics)
{
  if(diagnostics)
  {
    *diagnostics = HandState::VelocityDiagnostics{};
    diagnostics->targetPositionWorld = target.translation();
  }

  // Vive 输入约 100 Hz，而控制器运行在 500 Hz。只有收到新的 Vive/waist 样本时才做差分，
  // 并在两个样本之间保持结果，避免把阶梯位姿变成“一帧尖峰、四帧为零”的速度命令。
  if(previousTargetValid && sampleTime <= previousTargetTime + 1e-9)
  {
    if(velocitySampleTimeout_ > 0.0 && now() - sampleTime > velocitySampleTimeout_)
    {
      heldTargetVelocity = sva::MotionVecd::Zero();
    }
    if(diagnostics)
    {
      diagnostics->linearWorld = heldTargetVelocity.linear();
      diagnostics->angularWorld = heldTargetVelocity.angular();
    }
    return heldTargetVelocity;
  }

  if(!previousTargetValid || sampleTime <= 0.0 || sampleTime < previousTargetTime)
  {
    previousTarget = target;
    previousTargetTime = sampleTime;
    previousTargetValid = true;
    heldTargetVelocity = sva::MotionVecd::Zero();
    return heldTargetVelocity;
  }

  const double dt = sampleTime - previousTargetTime;
  sva::MotionVecd velocity = sva::transformError(previousTarget, target) / std::max(1e-4, dt);
  if(!velocity.vector().allFinite())
  {
    velocity = sva::MotionVecd::Zero();
  }

  // 轻量低通只作用于速度，不修改已经校准好的目标位置和目标姿态。
  heldTargetVelocity = sva::MotionVecd(
      velocityFilterAlpha_ * velocity.angular() + (1.0 - velocityFilterAlpha_) * heldTargetVelocity.angular(),
      velocityFilterAlpha_ * velocity.linear() + (1.0 - velocityFilterAlpha_) * heldTargetVelocity.linear());
  previousTarget = target;
  previousTargetTime = sampleTime;
  if(diagnostics)
  {
    diagnostics->linearWorld = heldTargetVelocity.linear();
    diagnostics->angularWorld = heldTargetVelocity.angular();
    diagnostics->sampleValid = true;
  }
  return heldTargetVelocity;
}

void ViveHandBridge::updateVelocityDiagnostics(HandState & hand,
                                               const sva::PTransformd & target,
                                               const sva::MotionVecd & worldVelocity,
                                               const sva::MotionVecd & commandWorldVelocity)
{
  auto & diagnostics = hand.velocityDiagnostics;
  diagnostics.targetPositionWorld = target.translation();
  diagnostics.linearWorld = worldVelocity.linear();
  diagnostics.angularWorld = worldVelocity.angular();
  diagnostics.linearCommandWorldRequested = commandWorldVelocity.linear();
  diagnostics.angularCommandWorldRequested = commandWorldVelocity.angular();

  // 这里复刻 TransformTask::targetVel 的内部换算：refVelB(frame.position() * worldVel)。
  // 再用同一个 task frame 反变换回 world，用红色箭头显示真正进入 task 后等效的 world 速度。
  const sva::PTransformd taskPose = hand.task ? hand.task->frame().position() : target;
  const sva::MotionVecd commandLocal = taskPose * commandWorldVelocity;
  diagnostics.linearCommandLocal = commandLocal.linear();
  diagnostics.angularCommandLocal = commandLocal.angular();
  const sva::MotionVecd commandWorldFromTask = taskPose.inv() * commandLocal;
  diagnostics.linearCommandWorldTaskFrame = commandWorldFromTask.linear();
  diagnostics.angularCommandWorldTaskFrame = commandWorldFromTask.angular();

  auto compare = [this](const Eigen::Vector3d & reference,
                        const Eigen::Vector3d & command,
                        double & cosine,
                        double & magnitudeRatio) {
    const double referenceNorm = reference.norm();
    const double commandNorm = command.norm();
    if(referenceNorm < velocityValidationMinSpeed_ || commandNorm < velocityValidationMinSpeed_)
    {
      cosine = 0.0;
      magnitudeRatio = 0.0;
      return false;
    }
    cosine = std::max(-1.0, std::min(1.0, reference.dot(command) / (referenceNorm * commandNorm)));
    magnitudeRatio = commandNorm / referenceNorm;
    return true;
  };

  diagnostics.linearComparisonValid =
      diagnostics.sampleValid
      && compare(diagnostics.linearWorld,
                 diagnostics.linearCommandWorldTaskFrame,
                 diagnostics.linearDirectionCosine,
                 diagnostics.linearMagnitudeRatio);
  diagnostics.angularComparisonValid =
      diagnostics.sampleValid
      && compare(diagnostics.angularWorld,
                 diagnostics.angularCommandWorldTaskFrame,
                 diagnostics.angularDirectionCosine,
                 diagnostics.angularMagnitudeRatio);
}

void ViveHandBridge::setupVelocityValidation(mc_control::fsm::Controller & ctl)
{
  if(!velocityValidation_)
  {
    return;
  }

  mc_rtc::gui::ArrowConfig worldArrow(mc_rtc::gui::Color::Green);
  mc_rtc::gui::ArrowConfig commandArrow(mc_rtc::gui::Color::Red);
  worldArrow.head_diam = commandArrow.head_diam = 0.02;
  worldArrow.head_len = commandArrow.head_len = 0.06;
  worldArrow.shaft_diam = commandArrow.shaft_diam = 0.01;

  auto addGui = [this, &ctl, &worldArrow, &commandArrow](HandState & hand, const std::string & side) {
    auto * handPtr = &hand;
    ctl.gui()->addElement(
        this,
        {"Avatar", "Vive", "Velocity validation", side},
        mc_rtc::gui::Arrow(
            "Linear derivative velW (green)",
            worldArrow,
            [handPtr]() { return handPtr->velocityDiagnostics.targetPositionWorld; },
            [this, handPtr]() -> Eigen::Vector3d {
              const auto & d = handPtr->velocityDiagnostics;
              return (d.targetPositionWorld + velocityValidationArrowScale_ * d.linearWorld).eval();
            }),
        mc_rtc::gui::Arrow(
            "Linear targetVel reconstructed in world (red)",
            commandArrow,
            [handPtr]() { return handPtr->velocityDiagnostics.targetPositionWorld; },
            [this, handPtr]() -> Eigen::Vector3d {
              const auto & d = handPtr->velocityDiagnostics;
              return (d.targetPositionWorld
                      + velocityValidationArrowScale_ * d.linearCommandWorldTaskFrame)
                  .eval();
            }),
        mc_rtc::gui::Label("Direction cosine", [handPtr]() {
          return handPtr->velocityDiagnostics.linearDirectionCosine;
        }),
        mc_rtc::gui::Label("Linear comparison valid", [handPtr]() {
          return handPtr->velocityDiagnostics.linearComparisonValid;
        }),
        mc_rtc::gui::Arrow(
            "Angular derivative velW (green)",
            worldArrow,
            [handPtr]() { return handPtr->velocityDiagnostics.targetPositionWorld; },
            [this, handPtr]() -> Eigen::Vector3d {
              const auto & d = handPtr->velocityDiagnostics;
              return (d.targetPositionWorld + velocityValidationArrowScale_ * d.angularWorld).eval();
            }),
        mc_rtc::gui::Arrow(
            "Angular targetVel reconstructed in world (red)",
            commandArrow,
            [handPtr]() { return handPtr->velocityDiagnostics.targetPositionWorld; },
            [this, handPtr]() -> Eigen::Vector3d {
              const auto & d = handPtr->velocityDiagnostics;
              return (d.targetPositionWorld
                      + velocityValidationArrowScale_ * d.angularCommandWorldTaskFrame)
                  .eval();
            }),
        mc_rtc::gui::Label("Angular direction cosine", [handPtr]() {
          return handPtr->velocityDiagnostics.angularDirectionCosine;
        }),
        mc_rtc::gui::Label("Angular comparison valid", [handPtr]() {
          return handPtr->velocityDiagnostics.angularComparisonValid;
        }));
  };

  addGui(left_, "Left hand");
  addGui(right_, "Right hand");
  addVelocityValidationLogs(ctl, left_, "left");
  addVelocityValidationLogs(ctl, right_, "right");
  mc_rtc::log::success(
      "[{}] Velocity validation enabled: green=pose-derived velW, red=targetVel reconstructed through current task frame",
      name());
}

void ViveHandBridge::addVelocityValidationLogs(mc_control::fsm::Controller & ctl,
                                               HandState & hand,
                                               const std::string & side)
{
  auto * handPtr = &hand;
  const std::string prefix = name() + "_velocity-validation_" + side + "_";

  auto addVector = [this, &ctl, handPtr, &prefix](
                       const std::string & suffix,
                       Eigen::Vector3d HandState::VelocityDiagnostics::* member) {
    const std::string entry = prefix + suffix;
    ctl.logger().addLogEntry(entry, [handPtr, member]() -> const Eigen::Vector3d & {
      return handPtr->velocityDiagnostics.*member;
    });
    velocityValidationLogEntries_.push_back(entry);
  };
  auto addDouble = [this, &ctl, handPtr, &prefix](
                       const std::string & suffix,
                       double HandState::VelocityDiagnostics::* member) {
    const std::string entry = prefix + suffix;
    ctl.logger().addLogEntry(entry, [handPtr, member]() -> const double & {
      return handPtr->velocityDiagnostics.*member;
    });
    velocityValidationLogEntries_.push_back(entry);
  };
  auto addBool = [this, &ctl, handPtr, &prefix](
                     const std::string & suffix,
                     bool HandState::VelocityDiagnostics::* member) {
    const std::string entry = prefix + suffix;
    ctl.logger().addLogEntry(entry, [handPtr, member]() -> const bool & {
      return handPtr->velocityDiagnostics.*member;
    });
    velocityValidationLogEntries_.push_back(entry);
  };

  addVector("target-position-world", &HandState::VelocityDiagnostics::targetPositionWorld);
  addVector("linear-world", &HandState::VelocityDiagnostics::linearWorld);
  addVector("linear-command-world-requested", &HandState::VelocityDiagnostics::linearCommandWorldRequested);
  addVector("linear-command-local", &HandState::VelocityDiagnostics::linearCommandLocal);
  addVector("linear-command-world-task-frame",
            &HandState::VelocityDiagnostics::linearCommandWorldTaskFrame);
  addDouble("linear-direction-cosine", &HandState::VelocityDiagnostics::linearDirectionCosine);
  addDouble("linear-magnitude-ratio", &HandState::VelocityDiagnostics::linearMagnitudeRatio);
  addBool("linear-comparison-valid", &HandState::VelocityDiagnostics::linearComparisonValid);

  addVector("angular-world", &HandState::VelocityDiagnostics::angularWorld);
  addVector("angular-command-world-requested", &HandState::VelocityDiagnostics::angularCommandWorldRequested);
  addVector("angular-command-local", &HandState::VelocityDiagnostics::angularCommandLocal);
  addVector("angular-command-world-task-frame",
            &HandState::VelocityDiagnostics::angularCommandWorldTaskFrame);
  addDouble("angular-direction-cosine", &HandState::VelocityDiagnostics::angularDirectionCosine);
  addDouble("angular-magnitude-ratio", &HandState::VelocityDiagnostics::angularMagnitudeRatio);
  addBool("angular-comparison-valid", &HandState::VelocityDiagnostics::angularComparisonValid);
  addBool("sample-valid", &HandState::VelocityDiagnostics::sampleValid);
}

sva::MotionVecd ViveHandBridge::feedforwardVelocity(const sva::MotionVecd & velocity) const
{
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  if(useAngularVelocityFeedforward_)
  {
    angular = angularVelocityGain_ * velocity.angular();
  }
  if(useLinearVelocityFeedforward_)
  {
    linear = linearVelocityGain_ * velocity.linear();
  }
  return sva::MotionVecd(angular, linear);
}

void ViveHandBridge::publishLegacyPoseVelocity(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & posePub,
    const rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr & velocityPub,
    const sva::PTransformd & target,
    const sva::MotionVecd & velocity)
{
  if(!publishLegacyUnityTopics_ || !posePub || !velocityPub)
  {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = "world";
  pose.pose.position.x = target.translation().x();
  pose.pose.position.y = target.translation().y();
  pose.pose.position.z = target.translation().z();
  const Eigen::Quaterniond q(target.rotation());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  posePub->publish(pose);

  geometry_msgs::msg::AccelStamped accel;
  accel.header = pose.header;
  // Guillaume 原版接口用 AccelStamped 承载速度量；这里按字段名放 angular/linear。
  accel.accel.angular.x = velocity.angular().x();
  accel.accel.angular.y = velocity.angular().y();
  accel.accel.angular.z = velocity.angular().z();
  accel.accel.linear.x = velocity.linear().x();
  accel.accel.linear.y = velocity.linear().y();
  accel.accel.linear.z = velocity.linear().z();
  velocityPub->publish(accel);
}

void ViveHandBridge::updateGrippers(mc_control::fsm::Controller & ctl)
{
  if(!controlGrippers_)
  {
    return;
  }
  // 第一次 grip 上升沿之前不发送任何夹具目标，避免 controller 启动时夹具自行张开。
  // 使用整夹具 opening 接口，让 RobotModule 决定所有主动/从动关节的正确联动。
  auto update = [&](const HandState & hand, const std::string & gripperName) {
    if(!hand.hasData || !hand.gripperReady || !hand.gripperCommandActive || !ctl.robot().hasGripper(gripperName))
    {
      return;
    }
    auto & gripper = ctl.robot().gripper(gripperName);
    const double opening = hand.gripperClosed ? closedValue_ : openValue_;
    gripper.setTargetOpening(opening);
  };
  update(left_, leftGripper_);
  update(right_, rightGripper_);
}

void ViveHandBridge::publishAdhesionCommand()
{
  if(!controlMujocoAdhesion_ || !adhesionPub_)
  {
    return;
  }
  std_msgs::msg::Float64MultiArray msg;
  // data[0] 是左夹具，data[1] 是右夹具；数值直接对应 MuJoCo adhesion actuator ctrl。
  msg.data = {left_.gripperClosed ? adhesionClosedValue_ : adhesionOpenValue_,
              right_.gripperClosed ? adhesionClosedValue_ : adhesionOpenValue_};
  adhesionPub_->publish(msg);
}

void ViveHandBridge::publishGraspEvent(const std::string & hand,
                                       const std::string & event,
                                       double triggerValue)
{
  if(!graspEventPub_)
  {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "{\"hand\":\"" + hand + "\",\"event\":\"" + event + "\",\"trigger\":"
             + std::to_string(triggerValue) + ",\"sequence\":" + std::to_string(++graspEventSequence_) + "}";
  graspEventPub_->publish(msg);
  mc_rtc::log::info("[{}] 抓取事件: hand={}, event={}, trigger={:.3f}, sequence={}", name(), hand, event,
                    triggerValue, graspEventSequence_);
}

bool ViveHandBridge::resetPressed(HandState & hand) const
{
  if(!hand.hasData)
  {
    return false;
  }
  // 当前实验固定使用 menu 作为启动/复位键，避免位掩码或触摸板产生别名事件。
  return hand.latest.menu_button;
}

bool ViveHandBridge::gripperPressed(HandState & hand) const
{
  if(!hand.hasData)
  {
    return false;
  }
  // 当前实验固定使用 grip 切换夹具；trigger 专用于“已抓住”通知。
  return hand.latest.grip_button;
}

EXPORT_SINGLE_STATE("ViveHandBridge", ViveHandBridge)
