cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME ANAAvatarControllerRHPS1CornelVive)
set(PROJECT_DESCRIPTION "Demonstrator for the ANA XPRIZE avatar challenge")
set(PROJECT_URL "https://github.com/isri-aist/mc_ana_avatar_controller")

include(cmake/base.cmake)

project(${PROJECT_NAME} CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

# 所有运行时库也安装到融合前缀，避免 mc_rtc 的下游默认行为写入系统工作区。
set(MC_RTC_HONOR_INSTALL_PREFIX ON)
find_package(mc_rtc REQUIRED)
# mc_rtc 导出的默认控制器安装目录指向系统工作区。融合版本必须完整安装到
# 自己的前缀，避免覆盖当前稳定控制器和状态库。
set(MC_CONTROLLER_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/lib/mc_controller")
set(MC_CONTROLLER_LIBRARY_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/lib/mc_controller")
find_package(ForceConstrainedTransformTask REQUIRED)
find_package(mc_neuron_mocap_plugin REQUIRED)
find_package(lipm_walking_controller REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc was compiled without ROS support")
endif()
find_package(3rd_party_ros REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vr_interface_msgs REQUIRED)

add_subdirectory(src)
option(BUILD_CNOID_PROJECTS "构建 Choreonoid/OpenHRP 工程" OFF)
if(BUILD_CNOID_PROJECTS)
  add_subdirectory(cnoid)
endif()
add_subdirectory(plugins)
add_subdirectory(utils)

option(
  IS_SIMULATION
  "Defines if this code is used for simulations rather than for real experiments (default is OFF)"
  FALSE)
if(IS_SIMULATION)
  set(SIM_CONF "true")
else()
  set(SIM_CONF "false")
endif()

# Sake2 variants use one active joint per gripper. Dedicated LEAP variants
# below require all 16 active joints before any Vive grip command is accepted.
set(AVATAR_GRIPPER_PROFILE "sake2")
set(AVATAR_GRIPPER_EXPECTED_ACTIVE_JOINTS 1)

set(AVATAR_INITIAL_STATE "LIPMWalking::Initial")
set(AVATAR_INITIAL_STATE_OUTPUT "Standing")
set(AVATAR_DEMO_WALK_FSM_STATE "- ANA::WalkFSM")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting")
set(AVATAR_DEMO_STATE "ANA::Demo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, ANA::PrepareRHPS1ForVR, Auto]\n      - [ANA::PrepareRHPS1ForVR, OK, ${AVATAR_RETARGETTING_STATE}, Auto]")
set(AVATAR_EXTRA_CONFIG "")
set(AVATAR_PLUGINS
    "[footsteps_planner_plugin, ExternalFootstepPlannerPlugin, mocap_plugin, EmergencyButtonPlugin, FingersForceSensorsPlugin]"
)
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# 实体 RHPS1 的纯手动 VR 版本。保留 LIPM 站立稳定器，但不进入仿真中
# 使用绝对坐标的准备轨迹，也不启动意图识别或共享自主状态。
# RHPS1 hardware is already brought to half-sitting and calibrated before
# Start MC Control. Keep that measured posture instead of asking the walking
# controller to drive to its own half-sitting target a second time.
set(AVATAR_INITIAL_STATE "ANA::ManualInitial")
set(AVATAR_INITIAL_STATE_OUTPUT "Standing")
set(AVATAR_DEMO_WALK_FSM_STATE "- ANA::WalkFSM")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::Manual")
set(AVATAR_DEMO_STATE "ANA::ManualDemo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, Avatar::Retargetting::Manual, Auto]")
set(AVATAR_EXTRA_CONFIG "")
# mocap 插件仅提供旧 retargetting state 需要的兼容 datastore；实际输入来自 Vive bridge。
# EmergencyButtonPlugin 在该源码包中不是必需依赖；真机急停仍由 RHPS1 硬件安全链负责。
set(AVATAR_PLUGINS "[mocap_plugin]")
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_manual.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_manual.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# 实体 RHPS1 的四条件实验版本。Control PC 始终加载这一套 controller；
# Operator PC 通过 ROS 2 在运行时选择 0/1.5 秒延迟和辅助开/关。
set(AVATAR_INITIAL_STATE "ANA::ManualInitial")
set(AVATAR_INITIAL_STATE_OUTPUT "Standing")
set(AVATAR_DEMO_WALK_FSM_STATE "- ANA::WalkFSM")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::Experiment")
set(AVATAR_DEMO_STATE "ANA::ExperimentDemo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, Avatar::Retargetting::Experiment, Auto]")
set(AVATAR_EXTRA_CONFIG "")
# Experiment input comes exclusively from ViveHandBridge. The legacy retargetting
# states only create the hand/arm tasks and no longer need a live mocap plugin.
set(AVATAR_PLUGINS "[]")
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_experiment.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_experiment.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

set(AVATAR_INITIAL_STATE "Avatar::Initial")
set(AVATAR_INITIAL_STATE_OUTPUT "OK")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::HeadPlusLeft")
set(AVATAR_DEMO_STATE "ANA::Demo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, ANA::PrepareRHPS1ForVR, Auto]\n      - [ANA::PrepareRHPS1ForVR, OK, ${AVATAR_RETARGETTING_STATE}, Auto]")
set(AVATAR_DEMO_WALK_FSM_STATE "")
set(AVATAR_EXTRA_CONFIG "")
# 工程师版 retargetting state 初始化时仍从 mocap 插件读取采样频率。
# 插件保持离线，只提供兼容 datastore；Vive bridge 才是实际目标来源。
set(AVATAR_PLUGINS "[mocap_plugin]")
# "robots: ground: module: env/ground rhps1: init_pos: translation: [0.154693,
# 0.030944, 1.052131] rotation: [0.980860, -0.080516, -0.163517, 0.068512] ")
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_none.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_none.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# Physical RHPS1 with two LEAP Hands. Arm/head retargeting, experiment delay,
# shared autonomy, pause/resume and between-trial reset remain unchanged.
# LEAPHandPlugin only transports mc_rtc gripper commands to the hardware;
# finger open/close commands still come from the Vive grip buttons.
set(AVATAR_GRIPPER_PROFILE "leap")
set(AVATAR_GRIPPER_EXPECTED_ACTIVE_JOINTS 16)
set(AVATAR_INITIAL_STATE "ANA::ManualInitial")
set(AVATAR_INITIAL_STATE_OUTPUT "Standing")
set(AVATAR_DEMO_WALK_FSM_STATE "- ANA::WalkFSM")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::Manual")
set(AVATAR_DEMO_STATE "ANA::ManualDemo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, Avatar::Retargetting::Manual, Auto]")
set(AVATAR_PLUGINS "[mocap_plugin, LEAPHandPlugin]")
set(AVATAR_EXTRA_CONFIG [=[
LEAPHandPlugin:
  - name: "left_leap_hand"
    usb_port: "/dev/ttyUSB0"
    robot: "rhps1"
    gripper: "l_gripper"
    simulation: false
    baudrate: 4000000
  - name: "right_leap_hand"
    usb_port: "/dev/ttyUSB1"
    robot: "rhps1"
    gripper: "r_gripper"
    simulation: false
    baudrate: 4000000
]=])
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_leap_manual.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_leap_manual.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::Experiment")
set(AVATAR_DEMO_STATE "ANA::ExperimentDemo")
set(AVATAR_DEMO_FSM_TRANSITIONS
    "      - [Pause, OK, Avatar::Retargetting::Experiment, Auto]")
set(AVATAR_PLUGINS "[LEAPHandPlugin]")
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_leap_experiment.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}_leap_experiment.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# 本机安装的 ISMPC 接口早于工程师分支；当前融合只构建实际需要的 LIPM 与无行走版本。
set(VARIANTS lipm manual experiment none leap_manual leap_experiment)
set(ROBOTS rhps1 hrp4j hrp4cr hrp4)
foreach(VARIANT ${VARIANTS})
  if(${VARIANT} STREQUAL "lipm")
    set(CTL_SUFFIX "")
  else()
    set(CTL_SUFFIX _${VARIANT})
  endif()
  install(
    FILES "${PROJECT_SOURCE_DIR}/etc/EmergencyButtonPlugin.yaml"
    DESTINATION
      "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}/etc/${PROJECT_NAME}${CTL_SUFFIX}/plugins/"
  )
  install(
    FILES "${PROJECT_SOURCE_DIR}/etc/FingersForceSensorsPlugin.yaml"
    DESTINATION
      "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}/etc/${PROJECT_NAME}${CTL_SUFFIX}/plugins/"
  )
  install(
    FILES "${PROJECT_SOURCE_DIR}/etc/ROS.yaml"
    DESTINATION
      "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}/etc/${PROJECT_NAME}${CTL_SUFFIX}/plugins/"
  )
  foreach(ROBOT ${ROBOTS})
    install(
      FILES "${PROJECT_SOURCE_DIR}/etc/robots/${ROBOT}.yaml"
      DESTINATION
        "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}/${PROJECT_NAME}${CTL_SUFFIX}/")
  endforeach()
endforeach()
