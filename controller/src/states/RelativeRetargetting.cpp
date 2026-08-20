#include "RelativeRetargetting.h"

#include <mc_control/fsm/Controller.h>

#include <mc_rtc/io_utils.h>

#include <mc_tasks/MetaTaskLoader.h>

void RelativeRetargetting::configure(const mc_rtc::Configuration & config)
{

  config("datastoreRetargetingReferenceRobot", datastoreRetargetingReferenceRobot_);
  config("datastoreRetargetingReferenceOperator", datastoreRetargetingReferenceOperator_);
  config("scaling", scaling_);
  config("dimWeight", dimWeight_);
  config("dimStiffness", dimStiffness_);
  config("minStiffness", minStiffness_);
  config("maxStiffness", maxStiffness_);
  config("weight", weight_);
  config("ori_weight_factor", ori_weight_fact);
  config("linearStiffTimeThreshold", linearStiffTimeThreshold_);
  config("maxStiffTimeThreshold", maxStiffTimeThreshold_);

  config("motionPrediction_model_frequency", model_frequency_);
  config("motionPrediction_input_sequence_length", input_sequence_length_);
  config("motionPrediction_output_sequence_length", output_sequence_length_);
  config("motionPrediction_sequence_Target", data_sequence_name);

  config("admittance", admittance_);
  config("safety_damping", safety_damping_);
  config("force_limit", force_limit_);
  config("force_margin", force_margin_);

  config("admittance_zmp", admittance_zmp_);
  config("safety_damping_zmp", safety_damping_zmp_);
  config("force_limit_zmp", force_limit_zmp_);
  config("force_margin_zmp", force_margin_zmp_);

  config("softJumpSafetyDistance", softJumpSafetyDistance_);
  config("criticalJumpSafetyDistance", criticalJumpSafetyDistance_);
  config("maxHandDistanceFromReferenceSafety", maxHandDistanceFromReferenceSafety_);
  config("maxElbowDistanceFromChestSafety", maxElbowDistanceFromChestSafety_);

  config("always_active", active_if_possible_);

  config("vive_trackers", use_vive_trackers_);
  config("hand_ros_topic", hand_topic_);
  config("hand_vel_ros_topic", hand_vel_topic_);
  config("reference_ros_topic", reference_topic_);

  config("state_ros_topic", state_topic_);
  config("trigger_ros_topic", trigger_topic_);

  target_Frame_prediction_ = Motion_Prediction(model_frequency_, input_sequence_length_, output_sequence_length_,
                                               Eigen::MatrixXd::Zero(8, 8), Eigen::MatrixXd::Zero(8, 8));

  config_.load(config);
}

void RelativeRetargetting::start(mc_control::fsm::Controller & ctl)
{
  if(!config_.has("robot") || (config_.has("robot") && !config_("robot").has(ctl.robot().name())))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot {} defined in the YAML configuration", name(),
                                                     ctl.robot().name());
  }
  auto rConfig = config_("robot")(ctl.robot().name());
  rConfig("origin", originFrame_);
  rConfig("target", targetFrame_);
  // rConfig("originOffset", originFrameOffset_);
  // rConfig("targetOffset", targetFrameOffset_);
  rConfig("activeJoints", activeJoints_);
  rConfig("body_frame_relative_pose", X_body_frame_);

  auto & mocap_freq_func = ctl.datastore().get<std::function<int(void)>>("mocap_plugin::get_data_frequency");
  data_frequency_ = mocap_freq_func();
  mc_rtc::log::info("[{}] data freq {}", name(), data_frequency_);

  int part = 0;
  config_("mocap_end_eff_num", part);
  std::cout << name() << " End eff num " << part << std::endl;
  End_effect = static_cast<MoCap_Body_part>(part);
  // End_effect = part;

  if(ctl.datastore().get<bool>("UseROS"))
  {
    nh_ = ana_ros_node_handle();

    if(use_vive_trackers_)
    {

      hand_pose_sub_.subscribe(*nh_, hand_topic_);
      hand_pose_sub_.maxTime(maxTime_);

      hand_vel_sub_.subscribe(*nh_, hand_vel_topic_);
      hand_vel_sub_.maxTime(maxTime_);
    }
    state_trigger_sub_.subscribe(*nh_, trigger_topic_);
    state_trigger_sub_.maxTime(maxTime_);
    retargetting_state_pub_ = nh_->advertise<std_msgs::Bool>(state_topic_, 1);
    state_trigger_pub_ = nh_->advertise<std_msgs::Bool>(trigger_topic_, 1);
  }

  if(!ctl.robot().hasFrame(originFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] robot->origin (\"{}\") must be a valid body of robot {}",
                                                     name(), originFrame_, ctl.robot().name());
  }
  if(!ctl.robot().hasFrame(targetFrame_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] robot->target (\"{}\") must define a valid frame of robot {}", name(), targetFrame_, ctl.robot().name());
  }

  ctl.datastore().make_call(name() + "::toggle_walking_mode",
                            [this]()
                            {
                              if(hand_task_->weight() != 0) { hand_task_->weight(0); }
                              else { hand_task_->weight(weight_); }
                            });
  ctl.datastore().make_call(name() + "::active", [this]() -> bool { return active_; });
  ctl.datastore().make_call(name() + "::data_online", [this]() -> bool { return data_online_; });
  ctl.datastore().make_call(name() + "::activate_deactivate",
                            [this, &ctl]()
                            {
                              if(!active_) { user_activate(ctl); }
                              else { user_deactivate(ctl); }
                            });
  ctl.datastore().make_call(
      name() + "::activate_deactivate_prediction",
      [this, &ctl]()
      {
        Prediction_On_ = !Prediction_On_;
        if(!Compute_Prediction_)
        {
          mc_rtc::log::warning("[{}] Prediction is not computed, deactiving...", name());
          Prediction_On_ = false;
        }
        auto & gui = *ctl.gui();
        if(Compute_Prediction_ && Prediction_On_)
        {

          gui.addElement(this, {"Avatar", name()},
                         mc_rtc::gui::Point3D("Prediction Pose",
                                              mc_rtc::gui::PointConfig(mc_rtc::gui::Color(1, 0, 0), 0.015),
                                              [this]() { return Predicted_Pose_Seq_.back(); }),
                         mc_rtc::gui::Trajectory("Prediction Pose Trajectory",
                                                 mc_rtc::gui::LineConfig(mc_rtc::gui::Color(1., 0., 0.), 0.01,
                                                                         mc_rtc::gui::LineStyle::Solid),
                                                 [this]() { return Predicted_Pose_Seq_; }));
        }
        else
        {
          gui.removeElement({"Avatar", name()}, "Prediction Pose");
          gui.removeElement({"Avatar", name()}, "Prediction Pose Trajectory");
        }
      });

  frame_ = mc_rbdyn::Frame::make(name() + "ControlFrame", ctl.robot().frame(targetFrame_), X_body_frame_, false);

  hand_task_ = std::make_shared<mc_tasks::ForceConstrainedTransformTask>(ctl.robot().frame(name() + "ControlFrame"),
                                                                         minStiffness_, weight_);

  hand_task_->stiffness(dimStiffness_ * minStiffness_);

  if(activeJoints_.size())
  {
    mc_rtc::log::info("[{}] Active joints: [{}]", name(), mc_rtc::io::to_string(activeJoints_));
    hand_task_->selectActiveJoints(activeJoints_);
  }
  hand_task_->reset();
  hand_task_->dimWeight(dimWeight_);
  Eigen::Vector6d dof = Eigen::Vector6d::Zero();
  dof << 1, 1, 1, 1, 1, 1;

  sva::AdmittanceVecd s_d = sva::AdmittanceVecd(safety_damping_.vector());
  sva::AdmittanceVecd adm = sva::AdmittanceVecd(admittance_.vector());

  // upper_cstr_id_ = hand_task_->addLocalConstraint(dof, force_limit_, force_margin_, s_d, adm);
  // lower_cstr_id_ = hand_task_->addLocalConstraint(-dof, force_limit_, force_margin_, s_d, adm);

  sva::PTransformd X_hand_hand0 =
      sva::PTransformd(hand_task_->frame().position().translation()) * hand_task_->frame().position().inv();
  mc_rbdyn::FramePtr frame0 =
      mc_rbdyn::Frame::make(name() + "Hand_Frame0", ctl.robot().frame(targetFrame_), X_hand_hand0, false);

  upper_cstr_id_ = hand_task_->addFrameConstraint(*frame0.get(), dof, force_limit_, force_margin_, s_d, adm);
  lower_cstr_id_ = hand_task_->addFrameConstraint(*frame0.get(), -dof, force_limit_, force_margin_, s_d, adm);

  frame_zmp_ = mc_rbdyn::Frame::make(name() + "ZMP_frame");

  upper_cstr_id_zmp_ = hand_task_->addFrameConstraint(*frame_zmp_.get(), dof, force_limit_zmp_, force_margin_zmp_,
                                                      sva::AdmittanceVecd(safety_damping_zmp_.vector()),
                                                      sva::AdmittanceVecd(admittance_zmp_.vector()));
  lower_cstr_id_zmp_ = hand_task_->addFrameConstraint(*frame_zmp_.get(), -dof, force_limit_zmp_, force_margin_zmp_,
                                                      sva::AdmittanceVecd(safety_damping_zmp_.vector()),
                                                      sva::AdmittanceVecd(admittance_zmp_.vector()));

  // upper_cstr_id_ = hand_task_->addWorldConstraint(dof, force_limit_, force_margin_, s_d, adm);
  // lower_cstr_id_ = hand_task_->addWorldConstraint(-dof, force_limit_, force_margin_, s_d, adm);

  target_ = hand_task_->target();

  Predicted_Pose_Seq_.push_back(T_U_hand_pred);
  Pose_Seq_.push_back(Eigen::Vector3d::Zero());

  ctl.datastore().make<std::shared_ptr<mc_tasks::ForceConstrainedTransformTask>>(name() + "Task", hand_task_);

  ctl.solver().addTask(hand_task_);

  const auto X_0_chest_referenceRobot = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceRobot_);
  const auto X_0_hand = ctl.robot().frame(targetFrame_).position();

  X_chest_hand_ = X_0_hand * X_0_chest_referenceRobot.inv();

  vel_filter = mc_filter::LowPass<Eigen::Vector6d>(ctl.solver().dt(), 1. / 20.);
  vel_pred_filter = mc_filter::LowPass<Eigen::Vector3d>(ctl.solver().dt(), 1. / 20.);

  if(auto posture_override_cfg = rConfig.find("postureOverride"))
  {
    posture_override_ = mc_tasks::MetaTaskLoader::load<mc_tasks::PostureTask>(ctl.solver(), *posture_override_cfg);
    // 左右手状态各自持有一个 posture task，必须使用不同名称以避免 GUI/logger 冲突。
    posture_override_->name(name() + "::ShoulderPostureOverride");

    std::string posture_override_joint;
    rConfig("postureOverrideJoint", posture_override_joint);
    if(!posture_override_joint.empty())
    {
      if(ctl.robot().hasJoint(posture_override_joint))
      {
        // 只约束产生局部极小值的肩俯仰关节，避免 posture task 干扰其余关节。
        posture_override_->selectActiveJoints(ctl.solver(), {posture_override_joint});

        bool use_current_target = true;
        rConfig("postureOverrideUseCurrentTarget", use_current_target);
        if(use_current_target)
        {
          const auto joint_index = ctl.robot().jointIndexByName(posture_override_joint);
          const auto & current_q = ctl.robot().mbc().q[joint_index];
          if(!current_q.empty())
          {
            // 以准备动作结束时的实际肩角作为软参考，防止进入遥操作时跳到固定姿态。
            posture_override_->target({{posture_override_joint, current_q}});
            mc_rtc::log::info("[{}] Shoulder branch reference: {}={}", name(), posture_override_joint,
                              current_q.front());
          }
        }
      }
      else
      {
        mc_rtc::log::warning("[{}] Shoulder workaround joint '{}' does not exist", name(), posture_override_joint);
      }
    }
    rConfig("postureOverrideActive", posture_override_active_);
    if(posture_override_active_)
    {
      // RHPS1 肩部局部极小值 workaround：启动时把肩俯仰维持在指定解支路。
      ctl.solver().addTask(posture_override_);
      mc_rtc::log::warning("[{}] RHPS1 shoulder posture override enabled", name());
    }
  }

  createGUI(ctl);
  createUnityGUI(ctl);
  createLogs(ctl);

  // thread_on = true;
  Prediction_Computation_thread = std::thread(&RelativeRetargetting::motion_prediction_thread_loop, this);
  Prediction_Computation_thread.detach();

  auto & logger = ctl.logger();

  MC_RTC_LOG_HELPER(name() + "_retargetting_active", active_);

  output("OK");
}

void RelativeRetargetting::update_gui_trajectory(mc_control::fsm::Controller &)
{
  Pose_Seq_.push_back(target_.translation());
  Predicted_Pose_Seq_.push_back(predicted_target_.translation());
  if(Pose_Seq_.size() > data_frequency_)
  {
    Pose_Seq_.erase(Pose_Seq_.begin());
    Predicted_Pose_Seq_.erase(Predicted_Pose_Seq_.begin());
  }
}

void RelativeRetargetting::activate(mc_control::fsm::Controller & ctl)
{

  if(!active_ && data_online_ && !(ctl.datastore().get<bool>("Emergency")))
  {
    mc_rtc::log::info("[{}] Activated tracking for {}", name(), hand_task_->surface());
    if(first_active_)
    {
      // avoid having the offsets preloaded before retrieving data
      auto rConfig = config_("robot")(ctl.robot().name());
      rConfig("originOffset", originFrameOffset_);
      rConfig("targetOffset", targetFrameOffset_);
      first_active_ = false;
    }
    hand_control_activatedTimestep_ = 0;
    hand_task_->reset();
    active_ = true;
  }
  else { mc_rtc::log::warning("[{}] Failed to activate, data status {}", name(), data_online_); }
}

void RelativeRetargetting::deactivate(mc_control::fsm::Controller &)
{

  if(active_) // If the task was active, reset the target to the current pose once
  {
    mc_rtc::log::info("[{}] Deactivated tracking for {}, user stat is {}", name(), hand_task_->surface(), user_active_);
  }
  hand_task_->stiffness(minStiffness_ * dimStiffness_);
  hand_task_->refVelB(sva::MotionVecd::Zero());
  active_ = false;
  Prediction_On_ = false;
  hand_control_activatedTimestep_ = 0;
}

bool RelativeRetargetting::run(mc_control::fsm::Controller & ctl)
{
  // Automatically activate if the index trigger button is pressed
  if(ctl.datastore().get<bool>("Emergency")) { user_deactivate(ctl); }
  if(!ctl.datastore().has("mocap_plugin::online")) { return true; }

  measured_wrench_ = hand_task_->frame().wrench();

  // measured_wrench_ = hand_task_->surfacePose().inv().dualMul(hand_task_->frame().wrench());

  update_force_cstr(ctl, lower_cstr_id_, admittance_, force_limit_, force_margin_, safety_damping_);
  update_force_cstr(ctl, upper_cstr_id_, admittance_, force_limit_, force_margin_, safety_damping_);
  update_force_cstr(ctl, lower_cstr_id_zmp_, admittance_zmp_, force_limit_zmp_, force_margin_zmp_, safety_damping_zmp_);
  update_force_cstr(ctl, upper_cstr_id_zmp_, admittance_zmp_, force_limit_zmp_, force_margin_zmp_, safety_damping_zmp_);

  auto & mocap_seq_func =
      ctl.datastore().get<std::function<Eigen::MatrixXd(MoCap_Body_part, MoCap_Parameters, int, int)>>(
          "mocap_plugin::get_sequence");
  auto & mocap_pose_func =
      ctl.datastore().get<std::function<sva::PTransformd(MoCap_Body_part)>>("mocap_plugin::get_pose");
  auto & mocap_vel_func =
      ctl.datastore().get<std::function<sva::MotionVecd(MoCap_Body_part)>>("mocap_plugin::get_velocity");
  auto & mocap_acc_func =
      ctl.datastore().get<std::function<Eigen::Vector3d(MoCap_Body_part)>>("mocap_plugin::get_accel");
  auto & mocap_online = ctl.datastore().get<bool>("mocap_plugin::online");

  data_online_ = mocap_online;

  const auto X_0_chest_referenceRobot = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceRobot_);
  auto & ctl_ana = *ctl.datastore().get<mc_avatar::WalkingInterfacePtr>("WalkingInterface");
  Eigen::Vector3d zmp_target = ctl_ana.get_zmp_target();
  frame_zmp_->position(sva::PTransformd(ctl.robot().posW().rotation(), zmp_target));
  measured_wrench_zmp_ =
      frame_zmp_->position().dualMul(hand_task_->surfacePose().inv().dualMul(hand_task_->frame().wrench()));

  auto pose = mocap_pose_func(End_effect);
  auto vel = mocap_vel_func(End_effect);
  auto acc = mocap_acc_func(End_effect);
  bool trigger = trigger_;

  if(ctl.datastore().get<bool>("UseROS"))
  {
    state_trigger_sub_.tick(ctl.solver().dt());
    trigger = state_trigger_sub_.data().value();
  }

  if(use_vive_trackers_)
  {
    data_online_ = unity_online_ && ctl.datastore().get<bool>(datastoreRetargetingReferenceOperator_ + "Active");
    pose = X_u0_pose_;
    vel = V_pose_;
    if(ctl.datastore().get<bool>("UseROS"))
    {
      hand_pose_sub_.tick(ctl.solver().dt());
      hand_vel_sub_.tick(ctl.solver().dt());
      data_online_ = hand_pose_sub_.data().isValid();
      pose = hand_pose_sub_.data().value();
      vel = hand_vel_sub_.data().value();
      vel = sva::MotionVecd(-vel.angular(), vel.linear());
    }
  }

  if(trigger && trigger_on_count_ * ctl.timeStep > 1)
  {
    if(active_) { user_deactivate(ctl); }
    else { user_activate(ctl); }
    if(ctl.datastore().get<bool>("UseROS"))
    {
      std_msgs::Bool trigger_msg;
      trigger_msg.data = false;
      trigger_on_count_ = 0;
      state_trigger_pub_.publish(trigger_msg);
    }
    trigger_ = false;
  }
  trigger_on_count_++;

  // mc_rtc::log::critical("[{}] Maybe activate? data_online {} active_if_possible {} first_active {} active {}
  // user_active {}", name(), data_online_, active_if_possible_, first_active_, active_, user_active_);
  if(data_online_ && active_if_possible_ && !first_active_ && !active_ && user_active_) { activate(ctl); }

  std::chrono::duration<double, std::milli> dt_prediction =
      std::chrono::high_resolution_clock::now() - t_controller_prediction;

  if(data_online_ && !use_vive_trackers_ && Compute_Prediction_ && (1 / dt_prediction.count()) * 1e3 <= data_frequency_)
  {
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    PoseSeq = mocap_seq_func(End_effect, MoCap_Position, 60, model_frequency_);
    // std::cout << "[" << name() << "]" << "PoseSeq cols " << PoseSeq.cols() << std::endl;
    AccSeq = mocap_seq_func(End_effect, MoCap_Accelerated_Velocity, 60, model_frequency_);
    thread_compute_trigger = true;
    T_U_hand_pred = target_Frame_prediction_.get_prediction_Pos_coordinate(t_forward_);
    V_mw_hand_pred = target_Frame_prediction_.get_prediction_Vel_coordinate(t_forward_);
    t_controller_prediction = std::chrono::high_resolution_clock::now();
  }

  if(active_ && data_online_)
  {

    sva::PTransformd X_m0_hand_pred_(pose.rotation(), T_U_hand_pred);

    auto X_m0_hand = pose;
    auto X_m0_chest = ctl.datastore().get<sva::PTransformd>(datastoreRetargetingReferenceOperator_);
    // X_m0_chest = sva::PTransformd(X_m0_chest.rotation(),X_0_chest_referenceRobot.translation());
    auto V_hand_m0 = vel;

    sva::MotionVecd V_hand_chest = sva::PTransformd(X_m0_chest.rotation()) * V_hand_m0;
    Eigen::Vector3d V_handPred_chest = X_m0_chest.rotation() * V_mw_hand_pred;

    X_chest_hand_ = X_m0_hand * X_m0_chest.inv();
    X_chest_hand_pred_ = X_m0_hand_pred_ * X_m0_chest.inv();
    X_chest_hand_.translation() *= scaling_;
    X_chest_hand_pred_.translation() *= scaling_;

    if(hand_control_activatedTimestep_ * ctl.timeStep <= maxStiffTimeThreshold_) { hand_control_activatedTimestep_++; }

    if(hand_control_activatedTimestep_ * ctl.timeStep < linearStiffTimeThreshold_)
    {
      hand_task_->stiffness(minStiffness_ * dimStiffness_);
      // mc_rtc::log::info("[{}] min",name());
    }
    else
    {
      double ratio = (hand_control_activatedTimestep_ * ctl.timeStep - linearStiffTimeThreshold_)
                     / (maxStiffTimeThreshold_ - linearStiffTimeThreshold_);
      mc_filter::utils::clampInPlace(ratio, 0, 1);
      hand_task_->stiffness((minStiffness_ + ratio * (maxStiffness_ - minStiffness_)) * dimStiffness_);
      // mc_rtc::log::info("[{}] increasing",name());
    }
    sva::PTransformd X_0_hand_target =
        targetFrameOffset_ * X_chest_hand_ * originFrameOffset_ * X_0_chest_referenceRobot;

    sva::MotionVecd V_hand_0 = sva::PTransformd(originFrameOffset_.rotation()).inv() * V_hand_chest;
    Eigen::Vector3d V_handPred_0 = originFrameOffset_.rotation().transpose() * V_handPred_chest;

    vel_filter.update((sva::PTransformd(ctl.robot().surfacePose(targetFrame_).rotation()) * V_hand_0).vector());
    vel_pred_filter.update(ctl.robot().surfacePose(targetFrame_).rotation() * V_handPred_0);
    vel_target_ = vel_filter.eval();
    lin_vel_pred_target_ = vel_pred_filter.eval();

    ref_vel_ = sva::MotionVecd(vel_target_);

    sva::MotionVecd ref_vel_pred = sva::MotionVecd(Eigen::Vector3d::Zero(), lin_vel_pred_target_);

    if(Prediction_On_ && !use_vive_trackers_) { ref_vel_ = ref_vel_pred; }
  }
  else
  {
    ref_vel_ = sva::MotionVecd::Zero();
    deactivate(ctl);
  }

  target_ = targetFrameOffset_ * X_chest_hand_ * originFrameOffset_ * X_0_chest_referenceRobot;
  predicted_target_ = targetFrameOffset_ * X_chest_hand_pred_ * originFrameOffset_ * X_0_chest_referenceRobot;

  hand_task_->target(target_);
  hand_task_->refVelB(ref_vel_);
  if(ctl.datastore().get<bool>("UseROS"))
  {
    std_msgs::Bool state_msg;
    state_msg.data = active_;
    retargetting_state_pub_.publish(state_msg);
  }

  return true;
}

void RelativeRetargetting::motion_prediction_thread_loop()
{
  while(thread_on)
  {
    if(thread_compute_trigger)
    {
      target_Frame_prediction_.Compute_Prediction_Trajectory_(PoseSeq, AccSeq, 0.);
      thread_compute_trigger = false;
    }
    else { std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
  }
  std::cout << "[" << name() << "] "
            << "Prediction thread off" << std::endl;
}

void RelativeRetargetting::teardown(mc_control::fsm::Controller & ctl)
{
  if(posture_override_ && posture_override_active_)
  {
    ctl.solver().removeTask(posture_override_);
    posture_override_active_ = false;
  }
  ctl.gui()->removeElements(this);
  ctl.logger().removeLogEntries(this);
  thread_on = false;
  if(Prediction_Computation_thread.joinable()) { Prediction_Computation_thread.join(); }
}

void RelativeRetargetting::update_force_cstr(mc_control::fsm::Controller & ctl,
                                             size_t id,
                                             const sva::ForceVecd & admittance,
                                             const sva::ForceVecd & force_lim,
                                             const sva::ForceVecd & force_margin,
                                             const sva::MotionVecd & damping)
{
  hand_task_->constraintAdmittance(id, admittance.vector());
  hand_task_->constraintWrench(id, force_lim.vector());
  hand_task_->constraintMargin(id, force_margin.vector());
  hand_task_->constraintDamping(id, damping.vector());
}

void RelativeRetargetting::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  auto & mocap_online = ctl.datastore().get<bool>("mocap_plugin::online");

  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::Label("Data Online", [this]() -> const bool & { return data_online_; }));

  gui.addElement(this, {"Avatar", name()},
                 mc_rtc::gui::NumberInput(
                     "Scaling", [this]() { return scaling_; }, [this](double s) { scaling_ = s; }),
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; },
                     [this, &ctl]()
                     {
                       if(!active_) { activate(ctl); }
                       else { deactivate(ctl); }
                     }));
  if(posture_override_)
  {
    gui.addElement(this, {"Avatar", name()},
                   mc_rtc::gui::Checkbox(
                       "Posture override", [this]() { return posture_override_active_; },
                       [this, &ctl]()
                       {
                         posture_override_active_ = !posture_override_active_;
                         if(posture_override_active_) { ctl.solver().addTask(posture_override_); }
                         else { ctl.solver().removeTask(posture_override_); }
                       }));
  }

  gui.addElement(this, {"Avatar", name(), "Task"},
                 mc_rtc::gui::NumberInput(
                     "Min stiffness", [this]() { return minStiffness_; }, [this](double s) { minStiffness_ = s; }),
                 mc_rtc::gui::NumberInput(
                     "Max stiffness", [this]() { return maxStiffness_; }, [this](double s) { maxStiffness_ = s; }),
                 mc_rtc::gui::ArrayInput(
                     "Dim Stiffness", {"r", "p", "y", "x", "y", "z"},
                     [this]() -> const Eigen::Vector6d & { return dimStiffness_; },
                     [this](const Eigen::Vector6d & t) { dimStiffness_ = t; }),
                 mc_rtc::gui::NumberInput(
                     "Weight", [this]() { return hand_task_->weight(); }, [this](double w) { hand_task_->weight(w); }));
  gui.addElement(this, {"Avatar", name(), "Offsets"},

                 mc_rtc::gui::ArrayInput(
                     "Origin offset (translation) [m]", {"x", "y", "z"},
                     [this]() -> const Eigen::Vector3d & { return originFrameOffset_.translation(); },
                     [this](const Eigen::Vector3d & t) { originFrameOffset_.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "Origin offset (rotation) [deg]", {"r", "p", "y"},
                     [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(originFrameOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                     [this](const Eigen::Vector3d & rpy)
                     { originFrameOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));

  gui.addElement(this, {"Avatar", name(), "Offsets", "Target"},
                 mc_rtc::gui::ArrayInput(
                     "Target offset (translation) [m]", {"x", "y", "z"},
                     [this]() -> const Eigen::Vector3d & { return targetFrameOffset_.translation(); },
                     [this](const Eigen::Vector3d & t) { targetFrameOffset_.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "Target offset (rotation) [deg]", {"r", "p", "y"},
                     [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(targetFrameOffset_.rotation()) * 180. / mc_rtc::constants::PI; },
                     [this](const Eigen::Vector3d & rpy)
                     { targetFrameOffset_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }));

  gui.addElement(
      this, {"Avatar", name(), "Force Safety", "Local Frame"},
      //  mc_rtc::gui::Label("Measured Force", [this]()
      //   {
      //    return fmt::format("{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}",
      //     measured_wrench_.couple().x() , measured_wrench_.couple().y(), measured_wrench_.couple().z(),
      //     measured_wrench_.force().x() , measured_wrench_.force().y(), measured_wrench_.force().z());
      //   }),
      mc_rtc::gui::ArrayInput(
          "Force Measured", {"cx", "cy", "cz", "fx", "fy", "fz"},
          [this]() -> sva::ForceVecd { return measured_wrench_; }, [this](sva::ForceVecd f) {}),
      mc_rtc::gui::ArrayInput(
          "Force Limit", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> sva::ForceVecd { return force_limit_; },
          [this](sva::ForceVecd f) { force_limit_ = f; }),
      mc_rtc::gui::ArrayInput(
          "Force Margin", {"cx", "cy", "cz", "fx", "fy", "fz"},
          [this]() -> const sva::ForceVecd & { return force_margin_; },
          [this](sva::ForceVecd f) { force_margin_ = f; }),
      mc_rtc::gui::ArrayInput(
          "Damping", {"wx", "wy", "wz", "vx", "vy", "vz"},
          [this]() -> const sva::MotionVecd & { return safety_damping_; },
          [this](sva::MotionVecd m) { safety_damping_ = m; }),
      mc_rtc::gui::ArrayInput(
          "Admittance", {"cx", "cy", "cz", "fx", "fy", "fz"},
          [this]() -> const sva::ForceVecd & { return admittance_; }, [this](sva::ForceVecd f) { admittance_ = f; }));
  gui.addElement(this, {"Avatar", name(), "Force Safety", "ZMP Frame"},
                 //  mc_rtc::gui::Label("Measured Force", [this]()
                 //   {
                 //    return fmt::format("{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}",
                 //     measured_wrench_.couple().x() , measured_wrench_.couple().y(), measured_wrench_.couple().z(),
                 //     measured_wrench_.force().x() , measured_wrench_.force().y(), measured_wrench_.force().z());
                 //   }),
                 mc_rtc::gui::ArrayInput(
                     "Force Measured", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> sva::ForceVecd { return measured_wrench_zmp_; }, [this](sva::ForceVecd f) {}),
                 mc_rtc::gui::ArrayInput(
                     "Force Limit", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> sva::ForceVecd { return force_limit_zmp_; },
                     [this](sva::ForceVecd f) { force_limit_zmp_ = f; }),
                 mc_rtc::gui::ArrayInput(
                     "Force Margin", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ForceVecd & { return force_margin_zmp_; },
                     [this](sva::ForceVecd f) { force_margin_zmp_ = f; }),
                 mc_rtc::gui::ArrayInput(
                     "Damping", {"wx", "wy", "wz", "vx", "vy", "vz"},
                     [this]() -> const sva::MotionVecd & { return safety_damping_zmp_; },
                     [this](sva::MotionVecd m) { safety_damping_zmp_ = m; }),
                 mc_rtc::gui::ArrayInput(
                     "Admittance", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() -> const sva::ForceVecd & { return admittance_zmp_; },
                     [this](sva::ForceVecd f) { admittance_zmp_ = f; }));
}

void RelativeRetargetting::createUnityGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();

  gui.addElement(this, {"Avatar", "Unity", "Pose"},
                 mc_rtc::gui::Transform(
                     name(), [this]() -> const sva::PTransformd & { return X_u0_pose_; },
                     [this](const sva::PTransformd & pose) { X_u0_pose_ = pose; }));
  gui.addElement(this, {"Avatar", "Unity", "Velocity"},
                 mc_rtc::gui::ArrayInput(
                     name(), {"vx", "vy", "vz", "wx", "wy", "wz"},
                     [this]() -> const sva::MotionVecd & { return V_pose_; },
                     [this](sva::MotionVecd v) { V_pose_ = v; }));
  gui.addElement(this, {"Avatar", "Unity", "Trigger"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return trigger_; }, [this]() { trigger_ = !trigger_; }));
  gui.addElement(this, {"Avatar", "Unity", "State"},
                 mc_rtc::gui::Checkbox(
                     name(), [this]() -> const bool & { return active_; }, [this]() {}));
  gui.addElement(
      this, {"Avatar", "Unity", "Data State"},
      mc_rtc::gui::Checkbox(
          name(), [this]() -> const bool & { return unity_online_; }, [this]() { unity_online_ = !unity_online_; }));
}

void RelativeRetargetting::createLogs(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntry(name() + "_motion-prediction_pose-0",
                           [this]() -> const Eigen::Vector3d & { return Pose_Seq_.back(); });
  ctl.logger().addLogEntry(name() + "_motion-prediction_prediction-pose-0",
                           [this]() -> const Eigen::Vector3d & { return Predicted_Pose_Seq_.back(); });
  ctl.logger().addLogEntry(name() + "_motion-prediction_prediction-vel-mW",
                           [this]() -> Eigen::Vector3d
                           { return target_Frame_prediction_.get_prediction_Vel_coordinate(t_forward_); });
  ctl.logger().addLogEntry(name() + "_refVelTarget_vel", [this]() -> const Eigen::Vector6d & { return vel_target_; });
  ctl.logger().addLogEntry(name() + "_refVelTarget_pred-vel",
                           [this]() -> const Eigen::Vector3d & { return lin_vel_pred_target_; });

  ctl.logger().addLogEntry(name() + "_motion-prediction_t-forward", [this]() -> const double & { return t_forward_; });
}
EXPORT_SINGLE_STATE("RelativeRetargetting", RelativeRetargetting)
