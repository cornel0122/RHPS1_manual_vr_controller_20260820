#include "IntentRecognitionBridge.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/ForceConstrainedTransformTask.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <Eigen/Geometry>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>

namespace
{
template<typename T>
T & datastoreValue(mc_control::fsm::Controller & ctl, const std::string & key, const T & initial)
{
  if(!ctl.datastore().has(key))
  {
    return ctl.datastore().make<T>(key, initial);
  }
  return ctl.datastore().get<T>(key);
}

bool contains(const std::vector<std::string> & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}
} // 匿名命名空间

void IntentRecognitionBridge::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
  config("enabled", enabled_);
  config("backend", backendName_);
  config("topic", topic_);
  config("left_topic", leftTopic_);
  config("write_assistance_datastore", writeAssistanceDatastore_);
  config("allow_missing_frame", allowMissingFrame_);
  config("accepted_frames", acceptedFrames_);
  config("default_hand", defaultHand_);
  config("confidence_threshold", confidenceThreshold_);
  config("alpha_cap", alphaCap_);
  config("position_scale", positionScale_);
  config("left_hand_task", leftTaskKey_);
  config("right_hand_task", rightTaskKey_);
  config("left_robot_hand_frame", leftRobotHandFrame_);
  config("right_robot_hand_frame", rightRobotHandFrame_);
  config("hand_input_source", handInputSource_);
  config("publish_robot_inputs", publishRobotInputs_);
  config("use_robot_camera_transform", useRobotCameraTransform_);
  config("robot_camera_frame", robotCameraFrame_);
  config("camera_optical_frame_id", cameraOpticalFrameId_);
  config("camera_pose_topic", cameraPoseTopic_);
  config("left_hand_camera_topic", leftHandCameraTopic_);
  config("right_hand_camera_topic", rightHandCameraTopic_);
  config("primary_only_assistance", primaryOnlyAssistance_);
  config("primary_assistance_actions", primaryAssistanceActions_);
  config("map_primary_to_guillaume_grab", mapPrimaryToGuillaumeGrab_);
  config("assistance_object_allowlist", assistanceObjectAllowlist_);
  if(const auto aliases = config.find("assistance_object_aliases"))
  {
    const auto configuredAliases =
        mc_rtc::Configuration(*aliases).operator std::map<std::string, std::string>();
    assistanceObjectAliases_.clear();
    assistanceObjectAliases_.insert(configuredAliases.begin(), configuredAliases.end());
  }

  std::vector<double> rotation;
  config("source_to_robot_rotation", rotation);
  if(rotation.size() == 9)
  {
    sourceToRobotRotation_ << rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5],
        rotation[6], rotation[7], rotation[8];
  }
  else if(!rotation.empty())
  {
    mc_rtc::log::warning("[{}] source_to_robot_rotation 必须包含 9 个数，本次使用单位矩阵", name());
  }

  std::vector<double> translation;
  config("source_to_robot_translation", translation);
  if(translation.size() == 3)
  {
    sourceToRobotTranslation_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  }
  else if(!translation.empty())
  {
    mc_rtc::log::warning("[{}] source_to_robot_translation 必须包含 3 个数，本次使用零平移", name());
  }

  backend_ = backendFromString(backendName_);
  defaultHand_ = defaultHand_ == "l" ? "l" : "r";

  // RHPS1 使用同一份控制器配置切换识别后端，避免为了实验模式复制多份 YAML。
  if(const char * modeEnv = std::getenv("ANA_RHPS1_INTENT_MODE"))
  {
    const std::string mode = modeEnv;
    if(mode == "none")
    {
      enabled_ = false;
      writeAssistanceDatastore_ = false;
    }
    else if(mode == "guillaume")
    {
      enabled_ = true;
      backendName_ = "guillaume";
      backend_ = Backend::Guillaume;
      writeAssistanceDatastore_ = false;
    }
    else if(mode == "cornel-monitor")
    {
      enabled_ = true;
      backendName_ = "cornel";
      backend_ = Backend::Cornel;
      writeAssistanceDatastore_ = false;
    }
    else if(mode == "cornel-assist")
    {
      enabled_ = true;
      backendName_ = "cornel";
      backend_ = Backend::Cornel;
      writeAssistanceDatastore_ = true;
    }
    else
    {
      mc_rtc::log::warning("[{}] 未知 ANA_RHPS1_INTENT_MODE='{}'，保持 YAML 配置", name(), mode);
    }
  }
}

void IntentRecognitionBridge::start(mc_control::fsm::Controller & ctl)
{
  // 手部与相机 frame 必须来自当前机器人，不能把 HRP4CR 名称用于 RHPS1。
  if(config_.has("robot") && config_("robot").has(ctl.robot().name()))
  {
    const auto robotConfig = config_("robot")(ctl.robot().name());
    robotConfig("left_robot_hand_frame", leftRobotHandFrame_);
    robotConfig("right_robot_hand_frame", rightRobotHandFrame_);
    robotConfig("robot_camera_frame", robotCameraFrame_);
  }

  status_ = enabled_ ? "waiting" : "disabled";

  ctl.gui()->addElement(
      this, {"Shared_Autonomy", "Intent"},
      mc_rtc::gui::Label("Enabled", [this]() { return enabled_; }),
      mc_rtc::gui::Label("Backend", [this]() -> const std::string & { return backendName_; }),
      mc_rtc::gui::Label("Write assistance data", [this]() { return writeAssistanceDatastore_; }),
      mc_rtc::gui::Label("Status", [this]() -> const std::string & { return status_; }),
      mc_rtc::gui::Label("Current goal", [this]() -> const std::string & { return currentGoal_; }),
      mc_rtc::gui::Label("Assistance goal", [this]() -> const std::string & {
        return currentAssistanceGoal_;
      }),
      mc_rtc::gui::Label("Input frame", [this]() -> const std::string & { return currentFrame_; }),
      mc_rtc::gui::Label("Frame safe", [this]() { return packetFrameSafe_; }),
      mc_rtc::gui::Label("Confidence", [this]() { return currentConfidence_; }),
      mc_rtc::gui::Label("Alpha", [this]() { return currentAlpha_; }));

  // 同时记录“收到推测”和“允许写入辅助数据”，避免实验统计把二者混为一谈。
  const std::string logPrefix = name() + "_experiment_";
  ctl.logger().addLogEntry(logPrefix + "enabled", this, [this]() { return enabled_; });
  ctl.logger().addLogEntry(logPrefix + "backend", this, [this]() -> const std::string & {
    return backendName_;
  });
  ctl.logger().addLogEntry(logPrefix + "write-assistance-datastore", this,
                           [this]() { return writeAssistanceDatastore_; });
  ctl.logger().addLogEntry(logPrefix + "has-packet", this, [this]() { return hasPacket_; });
  ctl.logger().addLogEntry(logPrefix + "frame-safe", this, [this]() { return packetFrameSafe_; });
  ctl.logger().addLogEntry(logPrefix + "assistance-data-ready", this,
                           [this]() { return assistanceDataReady_; });
  ctl.logger().addLogEntry(logPrefix + "status", this, [this]() -> const std::string & {
    return status_;
  });
  ctl.logger().addLogEntry(logPrefix + "current-goal", this, [this]() -> const std::string & {
    return currentGoal_;
  });
  ctl.logger().addLogEntry(logPrefix + "assistance-goal", this, [this]() -> const std::string & {
    return currentAssistanceGoal_;
  });
  ctl.logger().addLogEntry(logPrefix + "input-frame", this, [this]() -> const std::string & {
    return currentFrame_;
  });
  ctl.logger().addLogEntry(logPrefix + "confidence", this, [this]() { return currentConfidence_; });
  ctl.logger().addLogEntry(logPrefix + "alpha", this, [this]() { return currentAlpha_; });

  if(!enabled_)
  {
    mc_rtc::log::info("[{}] 意图识别桥保持关闭；当前纯遥操作不会受到影响", name());
    return;
  }

  if(backend_ == Backend::Guillaume)
  {
    // Guillaume 后端由原 AssistedTeleoperationPlugin 直接写入 datastore，
    // 本状态只检查数据是否齐全，不重复计算其 landmark/HMM。
    mc_rtc::log::info("[{}] 使用 Guillaume datastore 后端", name());
    return;
  }

  if(!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    ownsRclcpp_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("ana_intent_recognition_bridge");
  intentSub_ = node_->create_subscription<std_msgs::msg::String>(
      topic_, 5, [this](const std_msgs::msg::String::SharedPtr msg) { onExternalIntent(msg, 'r'); });
  leftIntentSub_ = node_->create_subscription<std_msgs::msg::String>(
      leftTopic_, 5, [this](const std_msgs::msg::String::SharedPtr msg) { onExternalIntent(msg, 'l'); });
  if(publishRobotInputs_)
  {
    cameraPosePub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(cameraPoseTopic_, 5);
    leftHandCameraPub_ =
        node_->create_publisher<geometry_msgs::msg::PointStamped>(leftHandCameraTopic_, 5);
    rightHandCameraPub_ =
        node_->create_publisher<geometry_msgs::msg::PointStamped>(rightHandCameraTopic_, 5);
  }
  mc_rtc::log::success("[{}] {} 后端已订阅右手 {}、左手 {}", name(), backendName_, topic_, leftTopic_);
  mc_rtc::log::info("[{}] Cornel 手部意图输入来源：{}", name(), handInputSource_);
}

bool IntentRecognitionBridge::run(mc_control::fsm::Controller & ctl)
{
  if(!enabled_)
  {
    assistanceDataReady_ = false;
    output("OK");
    return true;
  }

  if(backend_ == Backend::Guillaume)
  {
    updateGuillaumeStatus(ctl);
    output("OK");
    return true;
  }

  if(node_)
  {
    rclcpp::spin_some(node_);
    publishRobotInputs(ctl);
  }

  if(!hasPacket_)
  {
    assistanceDataReady_ = false;
    status_ = "waiting for /goal_beliefs";
    output("OK");
    return true;
  }

  latestPacket_ = mergedExternalPacket();

  packetFrameSafe_ = frameIsSafe(ctl, latestPacket_);
  currentFrame_ = latestPacket_.frameId.empty() ? "<missing>" : latestPacket_.frameId;
  currentGoal_ = latestPacket_.goals.empty() ? "none" : latestPacket_.goals.front().name;
  currentAlpha_ = latestPacket_.alpha;
  currentConfidence_ = latestPacket_.confidence;

  if(!writeAssistanceDatastore_)
  {
    assistanceDataReady_ = false;
    status_ = "monitor only";
  }
  else if(!packetFrameSafe_)
  {
    assistanceDataReady_ = false;
    status_ = "blocked: unsafe frame";
    if(!warnedUnsafeFrame_)
    {
      mc_rtc::log::warning(
          "[{}] 收到的目标 frame='{}' 未获准写入机器人世界坐标；识别结果仅监视，不会驱动共享自主",
          name(), currentFrame_);
      warnedUnsafeFrame_ = true;
    }
  }
  else
  {
    warnedUnsafeFrame_ = false;
    assistanceDataReady_ = writePacket(ctl, latestPacket_);
    status_ = assistanceDataReady_ ? "assistance data ready" : "monitor: no primary assistance goal";
  }

  output("OK");
  return true;
}

void IntentRecognitionBridge::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
  intentSub_.reset();
  leftIntentSub_.reset();
  cameraPosePub_.reset();
  leftHandCameraPub_.reset();
  rightHandCameraPub_.reset();
  node_.reset();
  if(ownsRclcpp_)
  {
    rclcpp::shutdown();
  }
}

void IntentRecognitionBridge::onExternalIntent(const std_msgs::msg::String::SharedPtr msg, char hand)
{
  IntentPacket packet;
  std::string error;
  if(!parseExternalPacket(msg->data, packet, error))
  {
    status_ = "parse error";
    if(!warnedParse_)
    {
      mc_rtc::log::warning("[{}] 无法解析 {} 的意图消息：{}", name(), backendName_, error);
      warnedParse_ = true;
    }
    return;
  }

  warnedParse_ = false;
  for(auto & goal : packet.goals)
  {
    goal.hand = hand;
    goal.assistanceAlpha = packet.alpha;
  }
  if(hand == 'l')
  {
    latestLeftPacket_ = std::move(packet);
    hasLeftPacket_ = true;
  }
  else
  {
    latestRightPacket_ = std::move(packet);
    hasRightPacket_ = true;
  }
  hasPacket_ = true;
}

IntentRecognitionBridge::IntentPacket IntentRecognitionBridge::mergedExternalPacket() const
{
  IntentPacket merged;
  const auto append = [&merged](const IntentPacket & packet) {
    if(merged.frameId.empty())
    {
      merged.frameId = packet.frameId;
    }
    merged.alpha = std::max(merged.alpha, packet.alpha);
    merged.confidence = std::max(merged.confidence, packet.confidence);
    merged.goals.insert(merged.goals.end(), packet.goals.begin(), packet.goals.end());
  };
  if(hasRightPacket_)
  {
    append(latestRightPacket_);
  }
  if(hasLeftPacket_)
  {
    append(latestLeftPacket_);
  }
  return merged;
}

bool IntentRecognitionBridge::parseExternalPacket(const std::string & data,
                                                  IntentPacket & packet,
                                                  std::string & error) const
{
  try
  {
    const auto payload = mc_rtc::Configuration::fromData(data);
    packet.frameId = payload("frame_id", std::string{});
    packet.alpha = payload("alpha", -1.0);
    packet.confidence = payload("confidence", -1.0);

    bool parsed = parseObjectGoals(payload, packet);
    if(backend_ == Backend::Cornel && packet.goals.empty())
    {
      parsed = parseCornelGoalBeliefs(payload, packet) || parsed;
    }
    if(!parsed || packet.goals.empty())
    {
      error = "消息中没有可用的 objects/intents 或 goals_beliefs";
      return false;
    }

    finalizePacket(packet);
    return true;
  }
  catch(const std::exception & e)
  {
    error = e.what();
    return false;
  }
}

bool IntentRecognitionBridge::parseObjectGoals(const mc_rtc::Configuration & payload,
                                               IntentPacket & packet) const
{
  const auto objects = payload.find("objects");
  if(!objects || !objects->isObject())
  {
    return false;
  }

  for(const auto & objectName : objects->keys())
  {
    const auto object = (*objects)(objectName);
    Eigen::Vector3d position(object("x", 0.0), object("y", 0.0), object("z", 0.0));
    const bool hasPosition = object.find("x").has_value() && object.find("y").has_value()
                             && object.find("z").has_value();
    const std::string hand = object("hand", defaultHand_);

    const auto intents = object.find("intents");
    if(intents && intents->isObject())
    {
      for(const auto & action : intents->keys())
      {
        GoalCandidate goal;
        goal.name = normalizedGoalName(action, objectName);
        goal.object = objectName;
        goal.action = action;
        goal.belief = (*intents)(action);
        goal.position = position;
        goal.hasPosition = hasPosition;
        goal.hand = hand == "l" ? 'l' : 'r';
        packet.goals.push_back(std::move(goal));
      }
    }
    else
    {
      GoalCandidate goal;
      goal.name = normalizedGoalName("approach_or_grab", objectName);
      goal.object = objectName;
      goal.action = "approach_or_grab";
      goal.belief = object("belief", 0.0);
      goal.position = position;
      goal.hasPosition = hasPosition;
      goal.hand = hand == "l" ? 'l' : 'r';
      packet.goals.push_back(std::move(goal));
    }
  }
  return !packet.goals.empty();
}

bool IntentRecognitionBridge::parseCornelGoalBeliefs(const mc_rtc::Configuration & payload,
                                                     IntentPacket & packet) const
{
  const auto beliefs = payload.find("goals_beliefs");
  if(!beliefs || !beliefs->isObject())
  {
    return false;
  }

  for(const auto & goalName : beliefs->keys())
  {
    GoalCandidate goal;
    goal.name = normalizedGoalName(goalName, "");
    const auto left = goalName.find('(');
    const auto right = goalName.rfind(')');
    if(left != std::string::npos && right != std::string::npos && right > left + 1)
    {
      goal.action = goalName.substr(0, left);
      goal.object = goalName.substr(left + 1, right - left - 1);
    }
    else
    {
      goal.action = goalName;
    }
    goal.belief = (*beliefs)(goalName);
    goal.hand = defaultHand_.front();
    packet.goals.push_back(std::move(goal));
  }
  return !packet.goals.empty();
}

void IntentRecognitionBridge::finalizePacket(IntentPacket & packet) const
{
  packet.goals.erase(
      std::remove_if(packet.goals.begin(), packet.goals.end(),
                     [](const GoalCandidate & goal) { return !std::isfinite(goal.belief) || goal.belief < 0.0; }),
      packet.goals.end());

  const double sum = std::accumulate(packet.goals.begin(), packet.goals.end(), 0.0,
                                     [](double total, const GoalCandidate & goal) {
                                       return total + goal.belief;
                                     });
  if(sum > std::numeric_limits<double>::epsilon())
  {
    for(auto & goal : packet.goals)
    {
      goal.belief /= sum;
    }
  }

  std::sort(packet.goals.begin(), packet.goals.end(),
            [](const GoalCandidate & lhs, const GoalCandidate & rhs) { return lhs.belief > rhs.belief; });

  if(packet.confidence < 0.0)
  {
    if(packet.goals.size() <= 1)
    {
      packet.confidence = packet.goals.empty() ? 0.0 : 1.0;
    }
    else
    {
      double entropy = 0.0;
      for(const auto & goal : packet.goals)
      {
        if(goal.belief > 0.0)
        {
          entropy -= goal.belief * std::log(goal.belief);
        }
      }
      packet.confidence = std::clamp(1.0 - entropy / std::log(static_cast<double>(packet.goals.size())), 0.0,
                                     1.0);
    }
  }

  if(packet.alpha < 0.0)
  {
    packet.alpha = packet.confidence > confidenceThreshold_ ? std::min(alphaCap_, packet.confidence) : 0.0;
  }
  packet.alpha = std::clamp(packet.alpha, 0.0, alphaCap_);

  for(auto & goal : packet.goals)
  {
    if(goal.hasPosition && !useRobotCameraTransform_)
    {
      goal.position =
          sourceToRobotRotation_ * (positionScale_ * goal.position) + sourceToRobotTranslation_;
    }
  }
}

bool IntentRecognitionBridge::frameIsSafe(mc_control::fsm::Controller & ctl,
                                          const IntentPacket & packet) const
{
  if(packet.frameId.empty())
  {
    return allowMissingFrame_;
  }
  if(useRobotCameraTransform_ && packet.frameId == cameraOpticalFrameId_)
  {
    return ctl.realRobot().hasFrame(robotCameraFrame_) || ctl.robot().hasFrame(robotCameraFrame_);
  }
  return contains(acceptedFrames_, packet.frameId);
}

bool IntentRecognitionBridge::writePacket(mc_control::fsm::Controller & ctl,
                                          const IntentPacket & packet)
{
  std::vector<std::string> names;
  std::vector<double> probabilities;
  std::vector<char> hands;
  std::unordered_map<int, std::vector<double>> positions;
  std::unordered_map<int, std::vector<double>> orientations;
  std::unordered_map<std::string, bool> tableCollision;
  double leftAlpha = 0.0;
  double rightAlpha = 0.0;

  names.reserve(packet.goals.size());
  probabilities.reserve(packet.goals.size());
  hands.reserve(packet.goals.size());

  for(const auto & goal : packet.goals)
  {
    if(primaryOnlyAssistance_ && !isPrimaryAssistanceAction(goal.action))
    {
      continue;
    }
    const std::string assistanceObject = assistanceObjectName(goal.object);
    if(!isAllowedAssistanceObject(assistanceObject))
    {
      continue;
    }

    const int index = static_cast<int>(names.size());
    names.push_back(assistanceGoalName(goal));
    probabilities.push_back(goal.belief);
    hands.push_back(goal.hand);
    if(goal.hand == 'l')
    {
      leftAlpha = std::max(leftAlpha, goal.assistanceAlpha);
    }
    else
    {
      rightAlpha = std::max(rightAlpha, goal.assistanceAlpha);
    }
    Eigen::Vector3d robotPosition = goal.position;
    if(goal.hasPosition && useRobotCameraTransform_ && packet.frameId == cameraOpticalFrameId_)
    {
      robotPosition = cameraOpticalToWorld(ctl, positionScale_ * goal.position);
    }
    else if(goal.hasPosition)
    {
      robotPosition =
          sourceToRobotRotation_ * (positionScale_ * goal.position) + sourceToRobotTranslation_;
    }
    positions[index] = goal.hasPosition
                           ? std::vector<double>{robotPosition.x(), robotPosition.y(), robotPosition.z()}
                           : std::vector<double>{};
    // 外部识别器目前只提供目标位置；单位四元数让原 SharedAutonomy 保持其默认朝向策略。
    orientations[index] = {1.0, 0.0, 0.0, 0.0};
    if(!assistanceObject.empty())
    {
      tableCollision[assistanceObject] = true;
    }
  }

  currentAssistanceGoal_ = names.empty() ? "none" : names.front();
  if(names.empty())
  {
    return false;
  }

  datastoreValue(ctl, "goalsName", names) = names;
  datastoreValue(ctl, "goalNamesUI", names) = names;
  datastoreValue(ctl, "totalProbabilities", probabilities) = probabilities;
  datastoreValue(ctl, "goalProbabilities", probabilities) = probabilities;
  datastoreValue(ctl, "handToChoose", hands) = hands;
  datastoreValue(ctl, "mapOfGoalPositions", positions) = positions;
  datastoreValue(ctl, "mapOfGoalOrientations", orientations) = orientations;
  datastoreValue(ctl, "goalPredictionAlpha", 0.0) = packet.alpha;
  datastoreValue(ctl, "goalPredictionAlphaLeft", 0.0) = leftAlpha;
  datastoreValue(ctl, "goalPredictionAlphaRight", 0.0) = rightAlpha;
  datastoreValue(ctl, "IntentRecognitionBridgeDualHandActive", false) = true;
  datastoreValue(ctl, "currentPredictedGoal", 0) = 0;
  datastoreValue(ctl, "currentGoalName", std::string{"none"}) =
      names.empty() ? std::string{"none"} : names.front();
  datastoreValue(ctl, "needToWait", false) = false;
  datastoreValue(ctl, "tableCollision", tableCollision) = tableCollision;

  updateHandPositions(ctl);
  return true;
}

void IntentRecognitionBridge::publishRobotInputs(mc_control::fsm::Controller & ctl)
{
  const auto & observedRobot = ctl.realRobot().hasFrame(robotCameraFrame_) ? ctl.realRobot() : ctl.robot();
  if(!publishRobotInputs_ || !node_ || !observedRobot.hasFrame(robotCameraFrame_))
  {
    return;
  }

  const auto & X_0_camera = observedRobot.frame(robotCameraFrame_).position();
  Eigen::Matrix3d opticalToCamera;
  opticalToCamera << 0.0, 0.0, 1.0,
                    -1.0, 0.0, 0.0,
                     0.0, -1.0, 0.0;
  const Eigen::Matrix3d opticalToWorld = X_0_camera.rotation().transpose() * opticalToCamera;
  const auto stamp = node_->now();

  if(cameraPosePub_)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "world";
    pose.pose.position.x = X_0_camera.translation().x();
    pose.pose.position.y = X_0_camera.translation().y();
    pose.pose.position.z = X_0_camera.translation().z();
    const Eigen::Quaterniond q(opticalToWorld);
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    cameraPosePub_->publish(pose);
  }

  const auto publishHand = [&](const std::string & robotFrame,
                               const std::string & taskKey,
                               const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher) {
    if(!publisher)
    {
      return;
    }

    Eigen::Vector3d pointWorld;
    const bool preferManualTarget = handInputSource_ == "manual_task_target";
    if(preferManualTarget && ctl.datastore().has(taskKey))
    {
      // 使用 ViveHandBridge 已完成校准和延迟处理、但尚未叠加共享自主的人工目标。
      // 这样共享自主造成的机器人运动不会反过来生成新的意图 landmark。
      const auto & task =
          ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(taskKey);
      pointWorld = task->target().translation();
    }
    else if(observedRobot.hasFrame(robotFrame))
    {
      // 保留机器人实际手部位置作为旧实验和故障回退路径。
      pointWorld = observedRobot.frame(robotFrame).position().translation();
    }
    else if(ctl.datastore().has(taskKey))
    {
      const auto & task =
          ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(taskKey);
      pointWorld = task->target().translation();
    }
    else
    {
      return;
    }

    const Eigen::Vector3d point = worldToCameraOptical(ctl, pointWorld);
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = cameraOpticalFrameId_;
    msg.point.x = point.x();
    msg.point.y = point.y();
    msg.point.z = point.z();
    publisher->publish(msg);
  };

  publishHand(leftRobotHandFrame_, leftTaskKey_, leftHandCameraPub_);
  publishHand(rightRobotHandFrame_, rightTaskKey_, rightHandCameraPub_);
}

Eigen::Vector3d IntentRecognitionBridge::cameraOpticalToWorld(
    mc_control::fsm::Controller & ctl,
    const Eigen::Vector3d & point) const
{
  const auto & observedRobot = ctl.realRobot().hasFrame(robotCameraFrame_) ? ctl.realRobot() : ctl.robot();
  const auto & X_0_camera = observedRobot.frame(robotCameraFrame_).position();
  const Eigen::Vector3d pointCamera(point.z(), -point.x(), -point.y());
  return X_0_camera.rotation().transpose() * pointCamera + X_0_camera.translation();
}

Eigen::Vector3d IntentRecognitionBridge::worldToCameraOptical(
    mc_control::fsm::Controller & ctl,
    const Eigen::Vector3d & point) const
{
  const auto & observedRobot = ctl.realRobot().hasFrame(robotCameraFrame_) ? ctl.realRobot() : ctl.robot();
  const auto & X_0_camera = observedRobot.frame(robotCameraFrame_).position();
  const Eigen::Vector3d pointCamera =
      X_0_camera.rotation() * (point - X_0_camera.translation());
  return Eigen::Vector3d(-pointCamera.y(), -pointCamera.z(), pointCamera.x());
}

bool IntentRecognitionBridge::isPrimaryAssistanceAction(const std::string & action) const
{
  return contains(primaryAssistanceActions_, action);
}

std::string IntentRecognitionBridge::assistanceObjectName(const std::string & object) const
{
  const auto alias = assistanceObjectAliases_.find(object);
  return alias == assistanceObjectAliases_.end() ? object : alias->second;
}

bool IntentRecognitionBridge::isAllowedAssistanceObject(const std::string & object) const
{
  return !object.empty()
         && (assistanceObjectAllowlist_.empty() || contains(assistanceObjectAllowlist_, object));
}

std::string IntentRecognitionBridge::assistanceGoalName(const GoalCandidate & goal) const
{
  if(mapPrimaryToGuillaumeGrab_ && isPrimaryAssistanceAction(goal.action) && !goal.object.empty())
  {
    return normalizedGoalName("grab", assistanceObjectName(goal.object));
  }
  return goal.name;
}

void IntentRecognitionBridge::updateHandPositions(mc_control::fsm::Controller & ctl)
{
  std::vector<double> hands(6, 0.0);
  bool valid = true;

  if(ctl.datastore().has(rightTaskKey_))
  {
    const auto & task =
        ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(rightTaskKey_);
    const auto p = task->target().translation();
    hands[0] = p.x();
    hands[1] = p.y();
    hands[2] = p.z();
  }
  else
  {
    valid = false;
  }

  if(ctl.datastore().has(leftTaskKey_))
  {
    const auto & task =
        ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(leftTaskKey_);
    const auto p = task->target().translation();
    hands[3] = p.x();
    hands[4] = p.y();
    hands[5] = p.z();
  }
  else
  {
    valid = false;
  }

  if(valid)
  {
    datastoreValue(ctl, "handsPositions", hands) = hands;
  }
}

void IntentRecognitionBridge::updateGuillaumeStatus(mc_control::fsm::Controller & ctl)
{
  const bool ready = ctl.datastore().has("goalPredictionAlpha")
                     && ctl.datastore().has("currentPredictedGoal")
                     && ctl.datastore().has("currentGoalName")
                     && ctl.datastore().has("totalProbabilities");
  status_ = ready ? "Guillaume datastore ready" : "waiting for Guillaume plugin";
  assistanceDataReady_ = ready;
  packetFrameSafe_ = ready;
  currentFrame_ = "mc_rtc datastore";
  if(ready)
  {
    currentGoal_ = ctl.datastore().get<std::string>("currentGoalName");
    currentAlpha_ = ctl.datastore().get<double>("goalPredictionAlpha");
    currentConfidence_ = currentAlpha_;
  }
}

std::string IntentRecognitionBridge::normalizedGoalName(const std::string & action,
                                                        const std::string & object)
{
  std::string name = action;
  if(!object.empty() && name.find(object) == std::string::npos)
  {
    name += "_" + object;
  }

  for(char & c : name)
  {
    if(!(std::isalnum(static_cast<unsigned char>(c)) || c == '_'))
    {
      c = '_';
    }
  }
  name.erase(std::unique(name.begin(), name.end(), [](char a, char b) { return a == '_' && b == '_'; }),
             name.end());
  while(!name.empty() && name.front() == '_')
  {
    name.erase(name.begin());
  }
  while(!name.empty() && name.back() == '_')
  {
    name.pop_back();
  }
  return name.empty() ? "unknown_goal" : name;
}

IntentRecognitionBridge::Backend IntentRecognitionBridge::backendFromString(const std::string & name)
{
  if(name == "walid")
  {
    return Backend::Walid;
  }
  if(name == "cornel")
  {
    return Backend::Cornel;
  }
  if(name != "guillaume")
  {
    mc_rtc::log::warning("[IntentRecognitionBridge] 未知后端 '{}'，回退到 guillaume", name);
  }
  return Backend::Guillaume;
}

EXPORT_SINGLE_STATE("IntentRecognitionBridge", IntentRecognitionBridge)
