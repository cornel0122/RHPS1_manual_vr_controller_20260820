#pragma once

#include <mc_control/fsm/State.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

struct UI_SharedAutonomy : mc_control::fsm::State
{

  void getUniqueFileName();
  void writeGoalsToFile(std::vector<std::string>& goalsName, const std::vector<double>& goalsBeliefs);
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::string buildHudText(mc_control::fsm::Controller & ctl);

  std::string fileName = "Delay";
  std::vector<double> gaze_data_;
  std::vector<double> goalProba;
  std::vector<std::string> formatProba;
  std::chrono::time_point<std::chrono::system_clock> timer;
  std::chrono::time_point<std::chrono::system_clock> timerLog;
  double completionTime = 0.0;
  bool firstTime = true;
  bool hudEnabled_ = true;
  bool ownsRclcpp_ = false;
  std::string hudTopic_ = "/ana/shared_autonomy/hud";
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hudPub_;
};
