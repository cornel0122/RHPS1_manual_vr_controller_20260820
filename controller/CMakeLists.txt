cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME ANAAvatarControllerRHPS1)
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
find_package(ForceConstrainedTransformTask)
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

set(AVATAR_INITIAL_STATE "LIPMWalking::Initial")
set(AVATAR_INITIAL_STATE_OUTPUT "Standing")
set(AVATAR_DEMO_WALK_FSM_STATE "- ANA::WalkFSM")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting")
set(AVATAR_EXTRA_CONFIG "")
set(AVATAR_PLUGINS
    "[footsteps_planner_plugin, ExternalFootstepPlannerPlugin, mocap_plugin, EmergencyButtonPlugin, FingersForceSensorsPlugin]"
)
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ANAAvatarControllerRHPS1.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ANAAvatarControllerRHPS1.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

set(AVATAR_INITIAL_STATE "Avatar::Initial")
set(AVATAR_INITIAL_STATE_OUTPUT "OK")
set(AVATAR_RETARGETTING_STATE "Avatar::Retargetting::HeadPlusLeft")
set(AVATAR_DEMO_WALK_FSM_STATE "")
set(AVATAR_EXTRA_CONFIG "")
# 工程师版 retargetting state 初始化时仍从 mocap 插件读取采样频率。
# 插件保持离线，只提供兼容 datastore；Vive bridge 才是实际目标来源。
set(AVATAR_PLUGINS "[mocap_plugin]")
# "robots: ground: module: env/ground rhps1: init_pos: translation: [0.154693,
# 0.030944, 1.052131] rotation: [0.980860, -0.080516, -0.163517, 0.068512] ")
configure_file(etc/ANAAvatarController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ANAAvatarControllerRHPS1_none.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ANAAvatarControllerRHPS1_none.yaml"
        DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

# 本机安装的 ISMPC 接口早于工程师分支；当前融合只构建实际需要的 LIPM 与无行走版本。
set(VARIANTS lipm none)
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
