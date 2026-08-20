#pragma once

#include "ANAAvatarController.h"

template<typename WalkingCtl>
struct WalkingInterfaceImpl : public mc_avatar::WalkingInterface
{
  static constexpr bool is_lipm = std::is_same_v<WalkingCtl, lipm_walking::Controller>;
  static constexpr bool is_none = std::is_same_v<WalkingCtl, mc_control::fsm::Controller>;

  // 本机的 ISMPC 版本与工程师分支不兼容；融合版本仅保留已安装且可验证的 LIPM 和无行走基类。
  static_assert(is_lipm || is_none, "Write WalkingInterfaceImpl to support another walking base");

  WalkingInterfaceImpl(ANAAvatarController<WalkingCtl> & ctl) : ctl_(ctl) {}

  bool is_walking() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.walkingState == lipm_walking::WalkingState::SingleSupport
             || ctl_.walkingState == lipm_walking::WalkingState::DoubleSupport;
    }
    if constexpr(is_none) { return false; }
    __builtin_unreachable();
  }

  bool is_double_support() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.walkingState == lipm_walking::WalkingState::DoubleSupport
             || ctl_.walkingState == lipm_walking::WalkingState::Standing;
    }
    if constexpr(is_none) { return true; }
    __builtin_unreachable();
  }

  bool is_stopping() final
  {
    if constexpr(is_lipm) { return is_walking() && ctl_.pauseWalking; }
    if constexpr(is_none) { return false; }
    __builtin_unreachable();
  }

  bool is_stopped() final
  {
    if constexpr(is_lipm) { return !is_walking(); }
    if constexpr(is_none) { return true; }
    __builtin_unreachable();
  }

  void start_stop_walking() final
  {
    if constexpr(is_lipm)
    {
      if(ctl_.walkingState == lipm_walking::WalkingState::Standing)
      {
        if(!ctl_.startWalking)
        {
          mc_rtc::log::success("Start walking");
          ctl_.pauseWalking = false;
          ctl_.startWalking = true;
        }
      }
      else
      {
        if(!ctl_.pauseWalking)
        {
          mc_rtc::log::success("Stop walking");
          ctl_.pauseWalking = true;
        }
      }
      return;
    }
    if constexpr(is_none)
    {
      mc_rtc::log::critical("Walking cannot be started without a walking controller as a base");
      return;
    }
    __builtin_unreachable();
  }

  Eigen::Vector3d get_planner_ref_vel() final
  {
    if constexpr(is_lipm)
    {
      if(ctl_.datastore().has("HybridPlanner::GetVelocity"))
      {
        auto & fn = ctl_.datastore().template get<std::function<Eigen::Vector3d()>>("HybridPlanner::GetVelocity");
        return fn();
      }
      return Eigen::Vector3d::Zero();
    }
    if constexpr(is_none) { return Eigen::Vector3d::Zero(); }
    __builtin_unreachable();
  }

  void set_planner_ref_vel(const Eigen::Vector3d & v) final
  {
    if constexpr(is_lipm)
    {
      if(ctl_.datastore().has("HybridPlanner::SetVelocity"))
      {
        auto & fn =
            ctl_.datastore().template get<std::function<void(const Eigen::Vector3d &)>>("HybridPlanner::SetVelocity");
        fn(v);
      }
      return;
    }
    if constexpr(is_none) { return; }
    __builtin_unreachable();
  }

  void set_torso_pitch(double p) final
  {
    if constexpr(is_lipm)
    {
      ctl_.stabilizer()->torsoPitch(p);
      return ctl_.plan.torsoPitch(p);
    }
    if constexpr(is_none) { return; }
    __builtin_unreachable();
  }

  double get_com_height() final
  {
    if constexpr(is_lipm) { return ctl_.plan.comHeight(); }
    if constexpr(is_none) { return ctl_.robot().com().z(); }
    __builtin_unreachable();
  }

  void set_com_height(double h) final
  {
    if constexpr(is_lipm) { return ctl_.plan.comHeight(h); }
    if constexpr(is_none) { return; }
    __builtin_unreachable();
  }

  std::string get_support_foot() final
  {
    if constexpr(is_lipm) { return ctl_.plan.supportContact().surfaceName; }
    if constexpr(is_none) { return "LeftFoot"; }
    __builtin_unreachable();
  }

  Eigen::Vector3d get_zmp_target() final
  {
    if constexpr(is_lipm) { return ctl_.stabilizer()->targetZMP(); }
    if constexpr(is_none) { return Eigen::Vector3d::Zero(); }
    __builtin_unreachable();
  }

  double next_ts() final
  {
    if constexpr(is_lipm) { return 0.0; }
    if constexpr(is_none) { return 0.0; }
    __builtin_unreachable();
  }

private:
  ANAAvatarController<WalkingCtl> & ctl_;
};
