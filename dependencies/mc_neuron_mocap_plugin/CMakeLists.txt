cmake_minimum_required(VERSION 3.15)

project(mc_neuron_mocap_plugin VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 兼容插件必须安装到当前 superbuild 前缀，不能写入 Control PC 的系统 mc_rtc。
set(MC_RTC_HONOR_INSTALL_PREFIX ON)
find_package(mc_rtc REQUIRED)
include(CMakePackageConfigHelpers)

option(AUTOLOAD_MC_NEURON_MOCAP_PLUGIN "自动加载 Neuron mocap 兼容插件" OFF)

add_library(mc_neuron_mocap_properties INTERFACE)
add_library(mc_neuron_mocap_plugin::mc_neuron_mocap_properties ALIAS mc_neuron_mocap_properties)
target_include_directories(
  mc_neuron_mocap_properties
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

set(plugin_SRC
    src/plugin.cpp
    src/MoCap.cpp
    src/ClientSocket.cpp
    src/Socket.cpp)
set(plugin_HDR
    include/mc_neuron_mocap_plugin/MoCap.h
    src/plugin.h
    src/Socket.h
    src/ClientSocket.h
    src/SocketException.h)

add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_neuron_mocap_properties)

configure_file(
  etc/mc_neuron_mocap_plugin.in.yaml
  "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_neuron_mocap_plugin.yaml")
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_neuron_mocap_plugin.yaml"
  DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS mc_neuron_mocap_properties
  EXPORT mc_neuron_mocap_pluginTargets)
install(
  EXPORT mc_neuron_mocap_pluginTargets
  FILE mc_neuron_mocap_pluginTargets.cmake
  NAMESPACE mc_neuron_mocap_plugin::
  DESTINATION lib/cmake/mc_neuron_mocap_plugin)

configure_package_config_file(
  cmake/mc_neuron_mocap_pluginConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mc_neuron_mocap_pluginConfig.cmake"
  INSTALL_DESTINATION lib/cmake/mc_neuron_mocap_plugin)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mc_neuron_mocap_pluginConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mc_neuron_mocap_pluginConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mc_neuron_mocap_pluginConfigVersion.cmake"
  DESTINATION lib/cmake/mc_neuron_mocap_plugin)
