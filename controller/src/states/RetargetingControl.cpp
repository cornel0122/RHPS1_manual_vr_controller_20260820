#include "RetargetingControl.h"

void RetargetingControl::configure(const mc_rtc::Configuration & config)
{

  config("vive_trackers", use_vive_trackers_);

  config("state_ros_topic", state_topic_);
  config("trigger_ros_topic", trigger_topic_);
  config("retargeting_states", states_);

  config_.load(config);
}

void RetargetingControl::start(mc_control::fsm::Controller & ctl)
{

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();
    state_trigger_sub_.subscribe(*nh_, trigger_topic_);
    state_trigger_sub_.maxTime(maxTime_);
    retargetting_state_pub_ = nh_->advertise<std_msgs::Bool>(state_topic_, 1);
    state_trigger_pub_ = nh_->advertise<std_msgs::Bool>(trigger_topic_, 1);
  }

  ctl.datastore().make_call(name() + "::active", [this]() -> bool { return active_; });
  ctl.datastore().make_call(name() + "::data_online", [this]() -> bool { return data_online_; });
  ctl.datastore().make_call(name() + "::activate_deactivate",
                            [this, &ctl]()
                            {
                              if(!active_) { activate(ctl); }
                              else { deactivate(ctl); }
                            });
  createGUI(ctl);
  createUnityGUI(ctl);
}

void RetargetingControl::activate(mc_control::fsm::Controller & ctl)
{
  for(const std::string & state : states_)
  {
    if(ctl.datastore().has(state + "::active"))
    {
      auto & activate_deactivate_func = ctl.datastore().get<std::function<void(void)>>(state + "::activate_deactivate");
      auto & active_func = ctl.datastore().get<std::function<bool(void)>>(state + "::active");
      if(!active_func()) { activate_deactivate_func(); }
    }
  }
}

void RetargetingControl::deactivate(mc_control::fsm::Controller & ctl)
{
  for(const std::string & state : states_)
  {
    if(ctl.datastore().has(state + "::active"))
    {
      auto & activate_deactivate_func = ctl.datastore().get<std::function<void(void)>>(state + "::activate_deactivate");
      auto & active_func = ctl.datastore().get<std::function<bool(void)>>(state + "::active");
      if(active_func()) { activate_deactivate_func(); }
    }
  }
}

bool RetargetingControl::run(mc_control::fsm::Controller & ctl)
{
  data_online_ = true;
  active_ = true;
  for(const std::string & state : states_)
  {
    if(ctl.datastore().has(state + "::data_online"))
    {
      auto & online_func = ctl.datastore().get<std::function<bool(void)>>(state + "::data_online");
      auto & active_func = ctl.datastore().get<std::function<bool(void)>>(state + "::active");
      data_online_ = data_online_ && online_func();
      active_ = active_ && active_func();
    }
  }
  bool trigger = trigger_;
  if(ctl.datastore().get<bool>("UseROS")) { state_trigger_sub_.tick(ctl.solver().dt()); }

  if(trigger && trigger_on_count_ * ctl.timeStep > 1)
  {
    if(active_) { deactivate(ctl); }
    else { activate(ctl); }
    trigger_ = false;
    if(ctl.datastore().get<bool>("UseROS"))
    {
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      trigger_on_count_ = 0;
      state_trigger_pub_.publish(trigger_msg);
    }
  }
  trigger_on_count_++;

  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Bool state_msg;
    state_msg.data = active_;
    retargetting_state_pub_.publish(state_msg);
  }

  output("OK");
  return true;
}

void RetargetingControl::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

void RetargetingControl::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();

  gui.addElement(this, {"Avatar"},
                 mc_rtc::gui::Label(name() + "Data Online", [this]() -> const bool & { return data_online_; }),
                 mc_rtc::gui::Checkbox(
                     name() + " Activated", [this]() { return active_; },
                     [this, &ctl]()
                     {
                       if(!active_) { activate(ctl); }
                       else { deactivate(ctl); }
                     }));

  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; },
                     [this, &ctl]()
                     {
                       if(!active_) { activate(ctl); }
                       else { deactivate(ctl); }
                     }));
}

void RetargetingControl::createUnityGUI(mc_control::fsm::Controller & ctl)
{

  auto & gui = *ctl.gui();
  gui.addElement(this, {"Avatar", "Unity", "Trigger"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return trigger_; }, [this]() { trigger_ = !trigger_; }));
  gui.addElement(this, {"Avatar", "Unity", "State"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return active_; }, [this]() {}));
}

EXPORT_SINGLE_STATE("RetargetingControl", RetargetingControl)
