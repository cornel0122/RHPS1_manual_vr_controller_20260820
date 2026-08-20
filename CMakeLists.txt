cmake_minimum_required(VERSION 3.15)

# 该仓库的顶层 CMakeLists 是 mc-rtc-superbuild extension 入口，
# 不是 controller 本身的构建入口。
set(RHPS1_MANUAL_VR_EXTENSION_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# Vive talker 发布的 ROS 2 自定义消息必须先安装，controller
# 配置阶段才能通过 find_package(vr_interface_msgs)。
if(NOT TARGET vr_interface_msgs)
  AddProject(
    vr_interface_msgs
    SOURCE_DIR
      "${RHPS1_MANUAL_VR_EXTENSION_ROOT}/ros2_ws/src/vr_interface_msgs"
    CMAKE_ARGS -DBUILD_TESTING=OFF
    DEPENDS mc_rtc
  )
endif()

# 旧 retargetting state 仍使用 Neuron mocap 的公共类型，并在启动时读取
# 它提供的兼容 datastore。实际姿态目标仍全部由 Vive bridge 写入。
if(NOT TARGET mc_neuron_mocap_plugin)
  AddProject(
    mc_neuron_mocap_plugin
    SOURCE_DIR
      "${RHPS1_MANUAL_VR_EXTENSION_ROOT}/dependencies/mc_neuron_mocap_plugin"
    CMAKE_ARGS
      -DBUILD_TESTING=OFF
      -DAUTOLOAD_MC_NEURON_MOCAP_PLUGIN=OFF
    DEPENDS mc_rtc
  )
endif()

# 研究所 Control PC 的定制 superbuild 通常已包含下列工程。
# 只对当前 superbuild 已注册的 target 建立显式依赖，以兼容
# nulspace-compliance 等现场分支的命名差异。
set(RHPS1_MANUAL_VR_CONTROLLER_DEPENDS mc_rtc vr_interface_msgs mc_neuron_mocap_plugin)
foreach(optional_dependency lipm_walking_controller)
  if(TARGET "${optional_dependency}")
    list(APPEND RHPS1_MANUAL_VR_CONTROLLER_DEPENDS "${optional_dependency}")
  endif()
endforeach()

if(NOT TARGET ANAAvatarControllerRHPS1_manual_project)
  AddProject(
    ANAAvatarControllerRHPS1_manual_project
    SOURCE_DIR "${RHPS1_MANUAL_VR_EXTENSION_ROOT}/controller"
    CMAKE_ARGS
      -DBUILD_TESTING=OFF
      -DINSTALL_DOCUMENTATION=OFF
      -DBUILD_CNOID_PROJECTS=OFF
    DEPENDS ${RHPS1_MANUAL_VR_CONTROLLER_DEPENDS}
  )
endif()
