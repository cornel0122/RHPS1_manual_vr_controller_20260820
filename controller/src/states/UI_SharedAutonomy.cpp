#include "UI_SharedAutonomy.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/io_utils.h>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>


void UI_SharedAutonomy::getUniqueFileName()
{
  // Guillaume 原版把 belief 记录到 Documents；这里保留相同位置，方便和旧实验对比。
  std::string username_str(getenv("USER"));
  std::filesystem::path baseFilePath = "/home/" + username_str + "/Documents/TeleopData/" + fileName + ".txt";
  std::filesystem::create_directories(baseFilePath.parent_path());

  int cpt = 1;

  while (std::filesystem::exists(baseFilePath))
  {
    std::filesystem::path newFilePath = baseFilePath.replace_filename(fileName + "_" + std::to_string(cpt) + ".txt");

    if (!std::filesystem::exists(newFilePath))
    {
      fileName = newFilePath.string();  // Return the full path as a string
      break;
    }

    cpt++;
  }

  fileName = baseFilePath.string();
}

void UI_SharedAutonomy::writeGoalsToFile(std::vector<std::string>& goalsName, const std::vector<double>& goalsBeliefs)
{
  std::ofstream logFile;
  logFile.open(fileName, std::ios::out | std::ios::app);

  if (!logFile.is_open()){
    std::cerr << "Error opening file: " << fileName << std::endl;
    return;
  }

  // 第一行只写 goal 名称，后续每行写一次 belief 快照。
  if(logFile.tellp() == 0){
    for (const std::string& goalName : goalsName){
      logFile << goalName << " ";
    }
    logFile << std::endl;
  }

  // Write the goalsBeliefs on a new line
  for (double belief : goalsBeliefs) {
    logFile << std::fixed << std::setprecision(3) << belief << " ";
  }
  logFile << std::endl;

  logFile.close();
}

void UI_SharedAutonomy::configure(const mc_rtc::Configuration & config) 
{
  config("hud_enabled", hudEnabled_);
  config("hud_topic", hudTopic_);
}

void UI_SharedAutonomy::start(mc_control::fsm::Controller & ctl)
{
  getUniqueFileName();
  if(!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    ownsRclcpp_ = true;
  }
  node_ = std::make_shared<rclcpp::Node>("ana_shared_autonomy_hud");
  hudPub_ = node_->create_publisher<std_msgs::msg::String>(hudTopic_, 1);

  // 共享自主的 intent 层未启动时，这些 datastore 可能不存在；先建立安全默认值。
  auto & goalsName = [&ctl]() -> std::vector<std::string> & {
    if(!ctl.datastore().has("goalsName"))
      return ctl.datastore().make<std::vector<std::string>>(
          "goalsName", std::vector<std::string>{"no_goal_data"});
    return ctl.datastore().get<std::vector<std::string>>("goalsName");
  }();
  auto & goalsBeliefs = [&ctl]() -> std::vector<double> & {
    if(!ctl.datastore().has("totalProbabilities"))
      return ctl.datastore().make<std::vector<double>>("totalProbabilities", std::vector<double>{0.0});
    return ctl.datastore().get<std::vector<double>>("totalProbabilities");
  }();
  writeGoalsToFile(goalsName, goalsBeliefs);

  gaze_data_ = {0.0, 0.0, 0.0, 0.0, -0.3, -1.0};
  if (!ctl.datastore().has("gazeVectors"))
    ctl.datastore().make<std::vector<double>>("gazeVectors", gaze_data_);

  auto & goalProba = [&ctl]() -> std::vector<double> & {
  if(!ctl.datastore().has("goalProbabilities"))
    return ctl.datastore().make<std::vector<double>>("goalProbabilities", std::vector<double>(9, 1.0/9));
  else
    return ctl.datastore().get<std::vector<double>>("goalProbabilities");
  }();

  auto & goalNamesUI = [&ctl]() -> std::vector<std::string> & {
  if(!ctl.datastore().has("goalNamesUI"))
    return ctl.datastore().make<std::vector<std::string>>("goalNamesUI", std::vector<std::string>{"Goal 1", "Goal 2", "Goal 3", "Goal 4", "Goal 5", "Goal 6", "Goal 7", "Goal 8", "Goal 9"});
  else
    return ctl.datastore().get<std::vector<std::string>>("goalNamesUI");
  }();

  auto & alpha = [&ctl]() -> double & {
  if(!ctl.datastore().has("goalPredictionAlpha"))
    return ctl.datastore().make<double>("goalPredictionAlpha", 0.0);
  else
    return ctl.datastore().get<double>("goalPredictionAlpha");
  }();

  auto & currentGoalName = [&ctl]() -> std::string & {
  if(!ctl.datastore().has("currentGoalName"))
    return ctl.datastore().make<std::string>("currentGoalName", "no_goal_data");
  else
    return ctl.datastore().get<std::string>("currentGoalName");
  }();

  auto & totalScore = [&ctl]() -> double & {
  if(!ctl.datastore().has("totalScore"))
    return ctl.datastore().make<double>("totalScore", 0.0);
  else
    return ctl.datastore().get<double>("totalScore");
  }();

  /*std::chrono::time_point<std::chrono::system_clock> timer;
  if(!ctl.datastore().has("completionTime"))
    timer = ctl.datastore().make<std::chrono::time_point<std::chrono::system_clock>>("completionTime", std::chrono::system_clock::now());
  else
    timer = ctl.datastore().get<std::chrono::time_point<std::chrono::system_clock>>("completionTime");
  std::chrono::duration<double> elapsedTime = std::chrono::system_clock::now() - timer;
  completionTime = elapsedTime.count();*/

  for(unsigned int i = 0; i < goalProba.size(); i++)
    formatProba.push_back("{:0.3f}");

  auto & gui = *ctl.gui();

  if(!ctl.datastore().has("ANA::SharedAutonomyHUD"))
  {
    ctl.datastore().make<std::string>("ANA::SharedAutonomyHUD", "SA: standby");
  }

  gui.addElement({"Avatar", "Unity", "Gaze"}, mc_rtc::gui::ArrayInput(
                   "GazeData", {"x1", "y1", "z1", "x2", "y2", "z2"},
                   [this]() -> const std::vector<double> & { return gaze_data_; },
                   [this](const std::vector<double> & v) { gaze_data_ = v; }));
  gui.addElement({"Shared_Autonomy"}, mc_rtc::gui::Table(
                   "Goal_Probabilities",
                   [&goalNamesUI]() -> std::vector<std::string> & { return {goalNamesUI}; },
                   [this]() -> std::vector<std::string> & { return formatProba; },
                   [&goalProba]() -> std::vector<std::vector<double>> { return {goalProba}; }));
  gui.addElement({"Shared_Autonomy"}, mc_rtc::gui::Label(
                   "Alpha", [&alpha]() -> const double & { return alpha; }));
  gui.addElement({"Shared_Autonomy"}, mc_rtc::gui::Label(
                   "Current goal", [&currentGoalName]() -> const std::string & { return currentGoalName; }));
  gui.addElement({"Shared_Autonomy"}, mc_rtc::gui::Label(
                   "Score", [&totalScore]() -> const double & { return totalScore; }));
  gui.addElement({"Shared_Autonomy"}, mc_rtc::gui::Label(
                   "Completion time", [this]() -> const double & { return completionTime; }));
  gui.addElement({"Shared_Autonomy"},
                 mc_rtc::gui::Checkbox("HUD enabled", [this]() { return hudEnabled_; },
                                       [this]() { hudEnabled_ = !hudEnabled_; }),
                 mc_rtc::gui::Label("HUD topic", [this]() -> const std::string & { return hudTopic_; }),
                 mc_rtc::gui::Label("HUD text", [&ctl]() -> const std::string & {
                   return ctl.datastore().get<std::string>("ANA::SharedAutonomyHUD");
                 }));

}

std::string UI_SharedAutonomy::buildHudText(mc_control::fsm::Controller & ctl)
{
  const auto & goalsName = ctl.datastore().get<std::vector<std::string>>("goalsName");
  const auto & goalsBeliefs = ctl.datastore().get<std::vector<double>>("totalProbabilities");
  const auto & currentGoalName = ctl.datastore().get<std::string>("currentGoalName");
  const auto & alpha = ctl.datastore().get<double>("goalPredictionAlpha");

  std::ostringstream ss;
  ss << "Shared autonomy: " << (alpha > 0.001 ? "assist" : "monitor") << "\n";
  ss << "Goal: " << currentGoalName << "\n";
  ss << "Alpha: " << std::fixed << std::setprecision(2) << alpha << "\n";

  std::vector<size_t> order(goalsBeliefs.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&goalsBeliefs](size_t a, size_t b) {
    return goalsBeliefs[a] > goalsBeliefs[b];
  });

  const size_t n = std::min<size_t>(3, order.size());
  for(size_t rank = 0; rank < n; ++rank)
  {
    const size_t i = order[rank];
    const std::string name = i < goalsName.size() ? goalsName[i] : ("goal_" + std::to_string(i));
    ss << rank + 1 << ". " << name << ": " << std::fixed << std::setprecision(2) << goalsBeliefs[i] << "\n";
  }
  return ss.str();
}

bool UI_SharedAutonomy::run(mc_control::fsm::Controller & ctl)
{
  if(node_)
  {
    rclcpp::spin_some(node_);
  }

  if(firstTime && ctl.datastore().has("ANA::Head::active") && ctl.datastore().call<bool>("ANA::Head::active"))
  {
    timerLog = std::chrono::system_clock::now(); 
    timer = std::chrono::system_clock::now(); 
    firstTime = false;
  }
  else if(!firstTime && !ctl.datastore().get<bool>("taskAchieved"))
  {
    std::chrono::duration<double> elapsedTime = std::chrono::system_clock::now() - timer;
    completionTime = elapsedTime.count();
    elapsedTime = std::chrono::system_clock::now() - timerLog;
    if(elapsedTime.count() >= 0.5)
    {
      timerLog = std::chrono::system_clock::now();
      auto & goalsName = ctl.datastore().get<std::vector<std::string>>("goalsName");
      auto & goalsBeliefs = ctl.datastore().get<std::vector<double>>("totalProbabilities");
      writeGoalsToFile(goalsName, goalsBeliefs);
    }
  }

  if(gaze_data_.size() == 6)
  {
    auto & gazeVectors = ctl.datastore().get<std::vector<double>>("gazeVectors");
    for(unsigned int i = 0; i < gaze_data_.size(); i++)
      gazeVectors[i] = gaze_data_[i];
  }

  const auto hudText = buildHudText(ctl);
  ctl.datastore().get<std::string>("ANA::SharedAutonomyHUD") = hudText;
  ctl.datastore().get<std::string>("ANA::HUD") = hudText;
  if(hudEnabled_ && hudPub_)
  {
    std_msgs::msg::String msg;
    msg.data = hudText;
    hudPub_->publish(msg);
  }
  output("OK");
  return true;
}

void UI_SharedAutonomy::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
  hudPub_.reset();
  node_.reset();
  // ROS2 context 由整套控制链共享；状态退出时不能关闭全局 context。
}

EXPORT_SINGLE_STATE("UI_SharedAutonomy", UI_SharedAutonomy)
