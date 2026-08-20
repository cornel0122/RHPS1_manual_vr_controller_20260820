#include "ViveHeadBridge.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

void ViveHeadBridge::configure(const mc_rtc::Configuration & config)
{
  config("topic", topic_);
  config("task", taskName_);
  config("enable", enable_);
  config("zero_request_key", zeroRequestKey_);
  config("require_external_zero", requireExternalZero_);
  config("head_rotation_axis_mode", headRotationAxisMode_);
  config("head_pitch_gain", headPitchGain_);
}

void ViveHeadBridge::start(mc_control::fsm::Controller & ctl)
{
  if(!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    ownsRclcpp_ = true;
  }

  node_ = std::make_shared<rclcpp::Node>("ana_vive_head_bridge");
  // 头显在 ViveSimpleInterface 里按 tracker 消息发布，这里只取旋转来控制机器人头部。
  hmdSub_ = node_->create_subscription<Tracker>(topic_, 10, [this](const Tracker::SharedPtr msg) { onHmd(msg); });

  if(ctl.datastore().has(taskName_))
  {
    task_ = ctl.datastore().get<std::shared_ptr<mc_tasks::TransformTask>>(taskName_);
  }
  else
  {
    mc_rtc::log::warning("[{}] Missing datastore task {}", name(), taskName_);
  }

  ctl.gui()->addElement(this, {"Avatar", "Vive"},
                        mc_rtc::gui::Button("Set head zero", [this]() { setZero(); }),
                        mc_rtc::gui::Checkbox("Head bridge enabled", [this]() { return enable_; },
                                              [this]() { enable_ = !enable_; }),
                        mc_rtc::gui::Label("HMD data", [this]() { return hasData_; }),
                        mc_rtc::gui::Label("HMD zeroed", [this]() { return zeroed_; }),
                        mc_rtc::gui::Label("Require external head zero", [this]() { return requireExternalZero_; }));

  if(!ctl.datastore().has(zeroRequestKey_))
  {
    // ViveHandBridge 在双手同时按复位键时会把这个标志置为 true。
    // 头部 bridge 在 run() 中消费这个请求并立刻清回 false。
    ctl.datastore().make<bool>(zeroRequestKey_, false);
  }

  mc_rtc::log::success("[{}] Subscribed to {}", name(), topic_);
  mc_rtc::log::info("[{}] Head rotation axis mode: {}", name(), headRotationAxisMode_);
  mc_rtc::log::info("[{}] Head pitch gain: {}", name(), headPitchGain_);
}

bool ViveHeadBridge::run(mc_control::fsm::Controller & ctl)
{
  rclcpp::spin_some(node_);

  if(ctl.datastore().has(zeroRequestKey_))
  {
    auto & zeroRequested = ctl.datastore().get<bool>(zeroRequestKey_);
    if(zeroRequested && hasData_ && task_)
    {
      mc_rtc::log::info("[{}] Consuming external head zero request", name());
      setZero();
      zeroRequested = false;
    }
  }

  if(!enable_ || !hasData_ || !task_)
  {
    output("OK");
    return true;
  }

  if(!zeroed_)
  {
    if(requireExternalZero_)
    {
      // 启动阶段等待 ViveHandBridge 的双手确认复位，避免 HMD 第一帧姿态被自动拿来当零点。
      output("OK");
      return true;
    }
    else
    {
      // 第一次收到有效头显数据时自动记录零点，之后按相对旋转驱动头部。
      setZero();
    }
  }

  sva::PTransformd target = zeroRobot_;
  const Eigen::Matrix3d currentHmd = viveRotation(latest_);
  // 模式 3 在按下复位时的 HMD 局部坐标中计算相对旋转，不再依赖房间朝向。
  Eigen::Matrix3d delta;
  if(headRotationAxisMode_ == 3)
  {
    delta = zeroHmd_.transpose() * currentHmd;
  }
  else
  {
    delta = currentHmd * zeroHmd_.transpose();
  }
  const Eigen::Matrix3d mappedDelta = mapRotationDeltaToRobot(delta);
  const Eigen::Matrix3d correctedDelta = applyHeadRotationAxisMode(mappedDelta, zeroRobot_.rotation());
  if(headRotationAxisMode_ == 3)
  {
    // correctedDelta 已在机器人头部零点的局部坐标中表达，因此在零点姿态右侧组合。
    target.rotation() = zeroRobot_.rotation() * correctedDelta;
  }
  else
  {
    target.rotation() = correctedDelta * zeroRobot_.rotation();
  }
  task_->target(target);
  task_->refVelB(sva::MotionVecd::Zero());

  output("OK");
  return true;
}

void ViveHeadBridge::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
  hmdSub_.reset();
  node_.reset();
  if(ownsRclcpp_)
  {
    rclcpp::shutdown();
  }
}

void ViveHeadBridge::onHmd(const Tracker::SharedPtr msg)
{
  latest_ = *msg;
  hasData_ = msg->pose_is_valid;
}

Eigen::Matrix3d ViveHeadBridge::viveRotation(const Tracker & msg) const
{
  Eigen::Matrix3d r;
  r << static_cast<double>(msg.pose_matrix[0]), static_cast<double>(msg.pose_matrix[1]),
      static_cast<double>(msg.pose_matrix[2]), static_cast<double>(msg.pose_matrix[4]),
      static_cast<double>(msg.pose_matrix[5]), static_cast<double>(msg.pose_matrix[6]),
      static_cast<double>(msg.pose_matrix[8]), static_cast<double>(msg.pose_matrix[9]),
      static_cast<double>(msg.pose_matrix[10]);
  return r;
}

Eigen::Matrix3d ViveHeadBridge::mapRotationDeltaToRobot(const Eigen::Matrix3d & delta) const
{
  if(headRotationAxisMode_ == 2 || headRotationAxisMode_ == 3)
  {
    // 两次朝向不同的 HMD 手持校准都显示，HMD 局部坐标中的轴关系始终为：
    // 物理 yaw   主要出现在 Vive +Y/-Y；
    // 物理 pitch 主要出现在 Vive +X/-X；
    // 物理 roll  主要出现在 Vive +Z/-Z。
    // 模式 3 面向 RHPS1：头部只有 yaw/pitch 两个自由度，因此丢弃无法实现的独立 roll。
    // 模式 2 保留原来的三轴行为，作为可随时回退的稳定版本。
    Eigen::AngleAxisd aa(delta);
    Eigen::Vector3d viveRotVec = aa.angle() * aa.axis();
    Eigen::Vector3d robotRotVec(headRotationAxisMode_ == 3 ? 0.0 : viveRotVec.z(), viveRotVec.x(), -viveRotVec.y());
    // 只放大点头方向，保持现有 yaw/roll 轴映射和头部零点不变。
    robotRotVec.y() *= headPitchGain_;
    const double angle = robotRotVec.norm();
    if(angle < 1e-9)
    {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, robotRotVec / angle).toRotationMatrix();
  }

  Eigen::Matrix3d viveToRobot;
  // 头显坐标系和机器人头部任务坐标系方向不同，这里把相对旋转换到机器人坐标。
  viveToRobot << 0, 0, -1, -1, 0, 0, 0, 1, 0;
  return viveToRobot * delta.transpose() * viveToRobot.transpose();
}

Eigen::Matrix3d ViveHeadBridge::applyHeadRotationAxisMode(const Eigen::Matrix3d & mappedDelta,
                                                          const Eigen::Matrix3d & zeroRobotRotation) const
{
  if(headRotationAxisMode_ == 1)
  {
    // 现场观察：头部 yaw 已正确，但 pitch 和 roll 互换。
    // 这个交换必须在机器人头部零点坐标系里做；在世界坐标里交换会交换错可见轴。
    const Eigen::Matrix3d localDelta = zeroRobotRotation.transpose() * mappedDelta * zeroRobotRotation;
    const Eigen::Vector3d ypr = localDelta.eulerAngles(2, 1, 0);
    const Eigen::AngleAxisd yaw(ypr.x(), Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch(ypr.z(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd roll(ypr.y(), Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d correctedLocalDelta = (yaw * pitch * roll).toRotationMatrix();
    return zeroRobotRotation * correctedLocalDelta * zeroRobotRotation.transpose();
  }
  return mappedDelta;
}

void ViveHeadBridge::setZero()
{
  if(!hasData_ || !task_)
  {
    mc_rtc::log::warning("[{}] Cannot zero head, data {}, task {}", name(), hasData_, static_cast<bool>(task_));
    return;
  }
  zeroHmd_ = viveRotation(latest_);
  zeroRobot_ = task_->target();
  zeroed_ = true;
  mc_rtc::log::success("[{}] Zeroed HMD/head orientation", name());
}

EXPORT_SINGLE_STATE("ViveHeadBridge", ViveHeadBridge)
