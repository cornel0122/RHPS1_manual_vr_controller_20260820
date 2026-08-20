#include "SharedAutonomy.h"
//#include "../ROSSubscriber.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/ForceConstrainedTransformTask.h>
#include <cmath>

void SharedAutonomy::configure(const mc_rtc::Configuration & config)
{
  config("enabled", enabled_);
  config("overwritten_Task", overwritten_Task);
  config("translationStiffness", stiffness_);
  config("orientationStiffness", stiffnessOrientationTask_);
  config_.load(config);
}

bool SharedAutonomy::sharedAutonomyDataReady(mc_control::fsm::Controller & ctl)
{
  if(!enabled_)
  {
    return false;
  }
  if(!ctl.datastore().has("goalPredictionAlpha") || !ctl.datastore().has("currentPredictedGoal")
     || !ctl.datastore().has("mapOfGoalPositions") || !ctl.datastore().has("mapOfGoalOrientations")
     || !ctl.datastore().has("handToChoose") || !ctl.datastore().has("handsPositions")
     || !ctl.datastore().has("currentGoalName") || !ctl.datastore().has(overwritten_Task))
  {
    return false;
  }

  const auto goal = ctl.datastore().get<int>("currentPredictedGoal");
  const auto & hands = ctl.datastore().get<std::vector<char>>("handToChoose");
  const auto & positions =
      ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalPositions");
  const auto & orientations =
      ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalOrientations");
  return goal >= 0 && static_cast<size_t>(goal) < hands.size() && positions.count(goal) && orientations.count(goal);
}

void SharedAutonomy::fallbackToManualRetargetting(mc_control::fsm::Controller & ctl)
{
  if(ctl.datastore().has(overwritten_Task))
  {
    auto & currentHandTask = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(overwritten_Task);
    currentHandTask->weight(800);
  }
  if(hand_taskTranslation)
  {
    hand_taskTranslation->weight(0);
    hand_taskTranslation->reset();
  }
  if(hand_taskOrientationDown)
  {
    hand_taskOrientationDown->weight(0);
    hand_taskOrientationDown->reset();
  }
}

void SharedAutonomy::addExperimentLogs(mc_control::fsm::Controller & ctl)
{
  const std::string prefix = name() + "_experiment_";
  ctl.logger().addLogEntry(prefix + "configured-enabled", this, [this]() { return enabled_; });
  ctl.logger().addLogEntry(prefix + "data-ready", this, [this]() { return dataReady_; });
  ctl.logger().addLogEntry(prefix + "selected-for-assistance", this, [this]() { return selectedForAssistance_; });
  ctl.logger().addLogEntry(prefix + "assistance-requested", this, [this]() { return assistanceRequested_; });
  ctl.logger().addLogEntry(prefix + "assistance-active", this, [this]() { return assistanceActive_; });
  ctl.logger().addLogEntry(
      prefix + "translation-active", this, [this]() { return translationAssistanceActive_; });
  ctl.logger().addLogEntry(
      prefix + "orientation-active", this, [this]() { return orientationAssistanceActive_; });
  ctl.logger().addLogEntry(prefix + "belief-alpha", this, [this]() { return alpha; });
  ctl.logger().addLogEntry(prefix + "effective-alpha", this, [this]() { return effectiveAlpha_; });
  ctl.logger().addLogEntry(prefix + "target-distance-m", this, [this]() { return targetDistance_; });
  ctl.logger().addLogEntry(prefix + "target-position-x", this, [this]() { return targetPosition.x(); });
  ctl.logger().addLogEntry(prefix + "target-position-y", this, [this]() { return targetPosition.y(); });
  ctl.logger().addLogEntry(prefix + "target-position-z", this, [this]() { return targetPosition.z(); });
  ctl.logger().addLogEntry(
      prefix + "active-duration-s", this, [this]() { return cumulativeActiveDuration_; });
  ctl.logger().addLogEntry(
      prefix + "current-episode-duration-s", this, [this]() { return currentEpisodeDuration_; });
  ctl.logger().addLogEntry(prefix + "active-episode-count", this, [this]() { return activeEpisodeCount_; });
  ctl.logger().addLogEntry(
      prefix + "current-goal", this, [this]() -> const std::string & { return currentGoalNameLog_; });
}

void SharedAutonomy::updateAssistanceMetrics(mc_control::fsm::Controller & ctl,
                                             bool active,
                                             bool translationActive,
                                             bool orientationActive,
                                             double effectiveAlpha)
{
  assistanceActive_ = active;
  translationAssistanceActive_ = translationActive;
  orientationAssistanceActive_ = orientationActive;
  effectiveAlpha_ = active ? effectiveAlpha : 0.0;

  if(active)
  {
    cumulativeActiveDuration_ += ctl.timeStep;
    currentEpisodeDuration_ += ctl.timeStep;
  }

  if(active && !previousAssistanceActive_)
  {
    activeEpisodeCount_++;
    currentEpisodeDuration_ = ctl.timeStep;
    mc_rtc::log::success("[{}] 共享自主开始生效：goal={}, alpha={:.3f}", name(), currentGoalNameLog_,
                         effectiveAlpha_);
  }
  else if(!active && previousAssistanceActive_)
  {
    mc_rtc::log::info("[{}] 共享自主停止：goal={}, 本段={:.3f}s，累计={:.3f}s", name(),
                      currentGoalNameLog_, currentEpisodeDuration_, cumulativeActiveDuration_);
    currentEpisodeDuration_ = 0.0;
  }

  previousAssistanceActive_ = active;
}

void SharedAutonomy::start(mc_control::fsm::Controller & ctl)
{
  if(!config_.has("robot") || (config_.has("robot") && !config_("robot").has(ctl.robot().name())))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot {} defined in the YAML configuration", name(),
                                                     ctl.robot().name());
  }
  auto rConfig = config_("robot")(ctl.robot().name());
  rConfig("target_frame", target_frame_);
  rConfig("activeJoints", activeJoints_);
  rConfig("body_frame_relative_pose", X_body_frame);
  arm_Task = overwritten_Task + "::active";
  overwritten_Task += "Task";
  ctl.datastore().make<bool>("enableTranslationTask" + name(), true);
  ctl.datastore().make<bool>("enableOrientationTask" + name(), true);

  alpha = 0.00;
  currentGoal = 0;
  //targetPositions = ctl.datastore().get<std::vector<double>>("targetPositions");
  //targetOrientations = ctl.datastore().get<std::vector<double>>("targetOrientations");
  if(ctl.datastore().has("mapOfGoalPositions"))
  {
    mapOfGoalPositions = ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalPositions");
  }
  if(ctl.datastore().has("mapOfGoalOrientations"))
  {
    mapOfGoalOrientations = ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalOrientations");
  }

  if (sharedAutonomyDataReady(ctl))
  {
    alpha = ctl.datastore().get<double>("goalPredictionAlpha");
    currentGoal = ctl.datastore().get<int>("currentPredictedGoal");
    handToChoose = ctl.datastore().get<std::vector<char>>("handToChoose")[currentGoal];
    if(mapOfGoalPositions[currentGoal].size() == 0)
    {
      auto & currentHandTaskTranslation = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(overwritten_Task)->target().translation();
      targetPosition = {currentHandTaskTranslation[0], currentHandTaskTranslation[1], currentHandTaskTranslation[2]};
    }
    else if(mapOfGoalPositions[currentGoal].size() == 3)
    {
      targetPosition = {mapOfGoalPositions[currentGoal][0], mapOfGoalPositions[currentGoal][1], mapOfGoalPositions[currentGoal][2]};
      targetOrientation = {mapOfGoalOrientations[currentGoal][0], mapOfGoalOrientations[currentGoal][1], mapOfGoalOrientations[currentGoal][2], mapOfGoalOrientations[currentGoal][3]};
    }
    else
    {
      unsigned int i = 0;
      int distanceIndex = 0;
      int orientationIndex = 0;
      double minDistance = 1000.0;
      auto & handsPositions = ctl.datastore().get<std::vector<double>>("handsPositions");
      if(handToChoose == "r")
      {
        while(i < mapOfGoalPositions[currentGoal].size()/3)
        {
          double distance3D = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][i*3] - handsPositions[0], 2) + std::pow(mapOfGoalPositions[currentGoal][i*3+1] - handsPositions[1], 2)
                            + std::pow(mapOfGoalPositions[currentGoal][i*3+2] - handsPositions[2], 2));
          if(distance3D <= minDistance)
          {
            minDistance = distance3D;
            distanceIndex = i*3;
            orientationIndex = i*4;
          }
          i++;
        }
      }
      else
      {
        while(i < mapOfGoalPositions[currentGoal].size()/3)
        {
          double distance3D = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][i*3] - handsPositions[3], 2) + std::pow(mapOfGoalPositions[currentGoal][i*3+1] - handsPositions[4], 2)
                            + std::pow(mapOfGoalPositions[currentGoal][i*3+2] - handsPositions[5], 2));
          if(distance3D <= minDistance)
          {
            minDistance = distance3D;
            distanceIndex = i*3;
            orientationIndex = i*4;
          }
          i++;
        }
      }
      targetPosition = {mapOfGoalPositions[currentGoal][distanceIndex], mapOfGoalPositions[currentGoal][distanceIndex+1], mapOfGoalPositions[currentGoal][distanceIndex+2]};
      targetOrientation = {mapOfGoalOrientations[currentGoal][orientationIndex], mapOfGoalOrientations[currentGoal][orientationIndex+1], mapOfGoalOrientations[currentGoal][orientationIndex+2], mapOfGoalOrientations[currentGoal][orientationIndex+3]};
    }
  }

  Eigen::Vector3d look_at_vector = config_("look_at_vector");
  Eigen::Vector3d look_down_vector = config_("look_down_vector");
  targetOrientationDown = config_("look_down_target_vector");
  frame_ = mc_rbdyn::RobotFrame::make(name() + "ControlFrame", ctl.robot().frame(target_frame_), X_body_frame, false);
  hand_taskTranslation = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(name() + "ControlFrame"), stiffness_, weight_);
  //hand_taskTranslation->stiffness(dimStiffness_ * stiffness_);
  //hand_taskOrientation = std::make_shared<mc_tasks::LookAtTask>(ctl.robot().frame(name() + "ControlFrame"), look_at_vector, stiffnessOrientationTask_, weightOrientationTask);
  hand_taskOrientationDown = std::make_shared<mc_tasks::VectorOrientationTask>(ctl.robot().frame(name() + "ControlFrame"), look_down_vector, stiffnessOrientationTask_, weightOrientationTask);

  if(activeJoints_.size())
  {
    mc_rtc::log::info("[{}] Active joints: [{}]", name(), mc_rtc::io::to_string(activeJoints_));
    hand_taskTranslation->selectActiveJoints(activeJoints_);
    //hand_taskOrientation->selectActiveJoints(activeJoints_);
    hand_taskOrientationDown->selectActiveJoints(activeJoints_);
  }

  hand_taskTranslation->reset();
  //hand_taskOrientation->reset();
  hand_taskOrientationDown->reset();
  dimWeight_[0] = 0.0;
  dimWeight_[1] = 0.0;
  dimWeight_[2] = 0.0;
  hand_taskTranslation->dimWeight(dimWeight_);
  ctl.datastore().make<std::shared_ptr<mc_tasks::TransformTask>>(name() + "TaskTranslation", hand_taskTranslation);
  ctl.datastore().make<std::shared_ptr<mc_tasks::LookAtTask>>(name() + "TaskOrientation", hand_taskOrientation);
  ctl.datastore().make<std::shared_ptr<mc_tasks::VectorOrientationTask>>(name() + "TaskOrientationDown", hand_taskOrientationDown);
  ctl.solver().addTask(hand_taskTranslation);
  //ctl.solver().addTask(hand_taskOrientation);
  ctl.solver().addTask(hand_taskOrientationDown);

  dataReady_ = false;
  selectedForAssistance_ = false;
  assistanceRequested_ = false;
  assistanceActive_ = false;
  translationAssistanceActive_ = false;
  orientationAssistanceActive_ = false;
  previousAssistanceActive_ = false;
  effectiveAlpha_ = 0.0;
  targetDistance_ = 0.0;
  cumulativeActiveDuration_ = 0.0;
  currentEpisodeDuration_ = 0.0;
  activeEpisodeCount_ = 0;
  currentGoalNameLog_ = "none";
  addExperimentLogs(ctl);
}

bool SharedAutonomy::run(mc_control::fsm::Controller & ctl)
{
  // intent/goal 层还没有启动时，共享自主保持打开但不介入，避免影响基础遥操作。
  dataReady_ = sharedAutonomyDataReady(ctl);
  if(!dataReady_)
  {
    if(!warnedMissingData_)
    {
      mc_rtc::log::warning("[{}] Shared autonomy is enabled, but intent/goal datastore is not ready; keeping manual retargetting.", name());
      warnedMissingData_ = true;
    }
    fallbackToManualRetargetting(ctl);
    selectedForAssistance_ = false;
    assistanceRequested_ = false;
    currentGoalNameLog_ = "none";
    targetDistance_ = 0.0;
    updateAssistanceMetrics(ctl, false, false, false, 0.0);
    output("OK");
    return true;
  }
  warnedMissingData_ = false;

  const std::string type_ = config_("type");
  const bool dualHandIntent = ctl.datastore().has("IntentRecognitionBridgeDualHandActive")
                              && ctl.datastore().get<bool>("IntentRecognitionBridgeDualHandActive");
  alpha = ctl.datastore().get<double>("goalPredictionAlpha");
  currentGoal = ctl.datastore().get<int>("currentPredictedGoal");
  if(dualHandIntent)
  {
    const auto & hands = ctl.datastore().get<std::vector<char>>("handToChoose");
    const auto & probabilities = ctl.datastore().get<std::vector<double>>("goalProbabilities");
    const char expectedHand = type_ == "l" ? 'l' : 'r';
    double bestBelief = -1.0;
    for(size_t i = 0; i < hands.size() && i < probabilities.size(); ++i)
    {
      if(hands[i] == expectedHand && probabilities[i] > bestBelief)
      {
        bestBelief = probabilities[i];
        currentGoal = static_cast<int>(i);
      }
    }
    const std::string alphaKey = expectedHand == 'l' ? "goalPredictionAlphaLeft" : "goalPredictionAlphaRight";
    if(ctl.datastore().has(alphaKey))
    {
      alpha = ctl.datastore().get<double>(alphaKey);
    }
  }
  mapOfGoalPositions = ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalPositions");
  mapOfGoalOrientations = ctl.datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalOrientations");
  //targetPosition = {targetPositions[currentGoal*3], targetPositions[currentGoal*3+1], targetPositions[currentGoal*3+2]};
  //targetOrientation = {targetOrientations[currentGoal*4], targetOrientations[currentGoal*4+1], targetOrientations[currentGoal*4+2], targetOrientations[currentGoal*4+3]};
  handToChoose = ctl.datastore().get<std::vector<char>>("handToChoose")[currentGoal];
  auto & enableTranslationTask = ctl.datastore().get<bool>("enableTranslationTask" + name());
  auto & enableOrientationTask = ctl.datastore().get<bool>("enableOrientationTask" + name());
  double minDistance = 1000.0;
  double gammaStiffness = stiffness_;
  double alphaStiffness = gammaStiffness*150;
  double betaStiffness = 1000.0;
  double newStiffness = gammaStiffness + alphaStiffness * exp(-betaStiffness*hand_taskTranslation->eval()[0]);
  //handToChoose = 'r';

  if(mapOfGoalPositions[currentGoal].size() == 0)
  {
    auto & currentHandTaskTranslation = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(overwritten_Task)->target().translation();
    targetPosition = {currentHandTaskTranslation[0], currentHandTaskTranslation[1], currentHandTaskTranslation[2]};
  }
  else if(mapOfGoalPositions[currentGoal].size() == 3)
  {
    targetPosition = {mapOfGoalPositions[currentGoal][0], mapOfGoalPositions[currentGoal][1], mapOfGoalPositions[currentGoal][2]};
    //targetOrientation = {mapOfGoalOrientations[currentGoal][0], mapOfGoalOrientations[currentGoal][1], mapOfGoalOrientations[currentGoal][2], mapOfGoalOrientations[currentGoal][3]};
    auto & handsPositions = ctl.datastore().get<std::vector<double>>("handsPositions");
    if(handToChoose == "r")
    {
      minDistance = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][0] - handsPositions[0], 2) + std::pow(mapOfGoalPositions[currentGoal][1] - handsPositions[1], 2)
                          + std::pow(mapOfGoalPositions[currentGoal][2] - handsPositions[2], 2));
    }
    else
    {
      minDistance = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][0] - handsPositions[3], 2) + std::pow(mapOfGoalPositions[currentGoal][1] - handsPositions[4], 2)
                          + std::pow(mapOfGoalPositions[currentGoal][2] - handsPositions[5], 2));
    }
  }
  else
  {
    unsigned int i = 0;
    unsigned int j = 0;
    int distanceIndex = 0;
    int orientationIndex =0;
    auto & handsPositions = ctl.datastore().get<std::vector<double>>("handsPositions");
    if(handToChoose == "r")
    {
      while(i < mapOfGoalPositions[currentGoal].size())
      {
        double distance3D = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][i] - handsPositions[0], 2) + std::pow(mapOfGoalPositions[currentGoal][i+1] - handsPositions[1], 2)
                          + std::pow(mapOfGoalPositions[currentGoal][i+2] - handsPositions[2], 2));
        if(distance3D < minDistance)
        {
          minDistance = distance3D;
          distanceIndex = i;
          orientationIndex = j;
        }
        i+=3;
        j+=4;
      }
    }
    else
    {
      while(i < mapOfGoalPositions[currentGoal].size())
      {
        double distance3D = std::sqrt(std::pow(mapOfGoalPositions[currentGoal][i] - handsPositions[3], 2) + std::pow(mapOfGoalPositions[currentGoal][i+1] - handsPositions[4], 2)
                          + std::pow(mapOfGoalPositions[currentGoal][i+2] - handsPositions[5], 2));
        if(distance3D < minDistance)
        {
          minDistance = distance3D;
          distanceIndex = i;
          orientationIndex = j;
        }
        i+=3;
        j+=4;
      }
    }
    targetPosition = {mapOfGoalPositions[currentGoal][distanceIndex], mapOfGoalPositions[currentGoal][distanceIndex+1], mapOfGoalPositions[currentGoal][distanceIndex+2]};
    //targetOrientation = {mapOfGoalOrientations[currentGoal][orientationIndex], mapOfGoalOrientations[currentGoal][orientationIndex+1], mapOfGoalOrientations[currentGoal][orientationIndex+2], mapOfGoalOrientations[currentGoal][orientationIndex+3]};
  }

  if(minDistance < limitDistanceTranslation)
    enableTranslationTask = false;
  else
    enableTranslationTask = true;
  if(minDistance < limitDistanceOrientation)
    enableOrientationTask = false;
  else
    enableOrientationTask = true;
  targetDistance_ = minDistance;

  //std::cout << "Target position: " << targetPosition << std::endl;
  //std::cout << "Target orientation: " << targetOrientation.w() << ", " << targetOrientation.x() << ", " << targetOrientation.y() << ", " << targetOrientation.z() << std::endl;

  const auto & goalNames = ctl.datastore().get<std::vector<std::string>>("goalsName");
  currentGoalNameLog_ = currentGoal >= 0 && static_cast<size_t>(currentGoal) < goalNames.size()
                            ? goalNames[static_cast<size_t>(currentGoal)]
                            : ctl.datastore().get<std::string>("currentGoalName");
  selectedForAssistance_ = type_.compare(handToChoose) == 0;
  // 原版由 ROS/Unity retargetting 的 active 状态放行；Vive 桥直接写 task，旧状态不会变为 true。
  // 保留原条件，同时用 Vive 的双手启动确认作为 ROS2 直连链路的放行条件。
  const std::string viveTrackingKey = "ANA::ViveHandBridge::tracking_started";
  const bool legacyRetargettingActive = ctl.datastore().call<bool>(arm_Task);
  const bool viveTrackingStarted = ctl.datastore().has(viveTrackingKey)
                                   && ctl.datastore().call<bool>(viveTrackingKey);
  const bool armActive = selectedForAssistance_ && alpha > 0.0
                         && (legacyRetargettingActive || viveTrackingStarted);
  assistanceRequested_ = selectedForAssistance_ && alpha > 0.0 && armActive;
  if(assistanceRequested_)
  {
    // RHPS1 的基础姿态令夹具伸出轴保持水平。瓶子、水壶和调料瓶均沿用该姿态，
    // 让夹口从侧面接近，其伸出方向与容器的竖直长轴垂直。
    Eigen::Vector3d newTargetOrientationDown = targetOrientationDown;
    auto & currentGoalName = currentGoalNameLog_;
    if(currentGoalName.compare("grab_potato1") == 0)
    {
      if(type_.compare("r") == 0)
      {
        targetPosition[0] -= 0.02;
        targetPosition[1] -= 0.02;
        //targetPosition[2] += 0.04;
      }
      else
      {
        targetPosition[0] -= 0.02;
        //targetPosition[1] += 0.05;
        //targetPosition[2] += 0.04;
      }
    }
    else if(currentGoalName.compare("grab_bottle") == 0)
    {
      if(type_.compare("r") == 0)
      {
        targetPosition[0] -= 0.03;
        targetPosition[1] += 0.02;
        //targetPosition[2] += 0.08;
      }
      else
      {
        targetPosition[0] -= 0.03;
        targetPosition[1] -= 0.02;
        //targetPosition[2] += 0.08;
      }
    }
    else if(currentGoalName.compare("grab_pitcher") == 0)
    {
      targetPosition[0] -= 0.1;
      //targetPosition[1] += 0.03;
      //targetPosition[2] += 0.01;
    }
    else if(currentGoalName.compare("grab_salt") == 0)
    {
      if(type_.compare("r") == 0)
      {
        targetPosition[0] -= 0.03;
        targetPosition[1] -= 0.02;
        //targetPosition[2] += 0.08;
      }
      else
      {
        targetPosition[0] -= 0.03;
        //targetPosition[1] += 0.12;
        //targetPosition[2] += 0.08;
      }
    }
    else if(currentGoalName.compare("grab_sauce") == 0)
    {
      targetPosition[0] -= 0.03;
      targetPosition[1] -= 0.02;
      //targetPosition[2] += 0.04;
    }
    /*else if(currentGoalName.compare("pour_salt") == 0)
    {
      targetPosition[0] -= 0.08;
      targetPosition[1] -= 0.15;
      targetPosition[2] += 0.085;
    }
    else if(currentGoalName.compare("pour_sauce") == 0)
    {
      targetPosition[0] -= 0.08;
      targetPosition[1] -= 0.15;
      targetPosition[2] += 0.085;
    }
    else if(currentGoalName.compare("pour_pitcher") == 0)
    {
      targetPosition[0] -= 0.06;
      targetPosition[1] += 0.15;
      targetPosition[2] += 0.15;
    }
    else if(currentGoalName.compare("pour_bottle") == 0)
    {
      if(type_.compare("r") == 0)
      {
        targetPosition[0] -= 0.02;
        targetPosition[1] -= 0.03;
        targetPosition[2] += 0.2;
      }
      else
      {
        targetPosition[0] -= 0.06;
        targetPosition[1] += 0.15;
        targetPosition[2] += 0.15;
      }
    }*/
    hand_taskTranslation->target(sva::PTransformd(targetPosition));
    hand_taskTranslation->stiffness(newStiffness);
    //hand_taskOrientation->target(targetPosition);
    hand_taskOrientationDown->targetVector(newTargetOrientationDown);
    //hand_taskTranslation->refVelB(sva::MotionVecd(ref_vel_.angular(), Eigen::Vector3d::Zero()));

    auto & currentHandTask = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(overwritten_Task);
    auto & needToWait = ctl.datastore().get<bool>("needToWait");
    bool needToWaitBis = needToWait;
    if(currentGoalName.find("grab") == std::string::npos)
    {
      needToWaitBis = false;
      enableTranslationTask = true;
      enableOrientationTask = true;
    }
    else
    {
      std::string objectName = currentGoalName.substr(currentGoalName.find('_')+1);
      auto & tableCollision = ctl.datastore().get<std::unordered_map<std::string, bool>>("tableCollision");
      if(!tableCollision[objectName])
        needToWaitBis = true;
    }

    if(!enableTranslationTask || needToWaitBis)
    {
      hand_taskTranslation->weight(0);
      currentHandTask->weight(800);
    }
    else
    {
      hand_taskTranslation->weight(800*alpha);
      currentHandTask->weight(800*(1-alpha));
    }
    if(!enableOrientationTask || needToWaitBis)
    {
      hand_taskOrientationDown->weight(0);
    }
    else
    {
      hand_taskOrientationDown->weight(800*alpha);
    }
    const bool translationActive = enableTranslationTask && !needToWaitBis && alpha > 0.0;
    const bool orientationActive = enableOrientationTask && !needToWaitBis && alpha > 0.0;
    updateAssistanceMetrics(
        ctl, translationActive || orientationActive, translationActive, orientationActive, alpha);
  }
  else
  {
    auto & currentHandTask = ctl.datastore().get<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(overwritten_Task);
    currentHandTask->weight(800);
    hand_taskTranslation->reset();
    //hand_taskOrientation->reset();
    hand_taskOrientationDown->reset();
    updateAssistanceMetrics(ctl, false, false, false, 0.0);
  } 

  output("OK");
  return true;
}

void SharedAutonomy::teardown(mc_control::fsm::Controller & ctl) {
  mc_rtc::log::info("[{}] 共享自主实验汇总：累计生效={:.3f}s，生效段数={}", name(),
                    cumulativeActiveDuration_, activeEpisodeCount_);
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
}

EXPORT_SINGLE_STATE("SharedAutonomy", SharedAutonomy)
