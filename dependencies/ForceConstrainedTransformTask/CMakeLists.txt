cmake_minimum_required(VERSION 3.15)

project(ForceConstrainedTransformTask LANGUAGES CXX VERSION 0.1.0)

# 该任务随 extension 安装，不能覆盖 Control PC 的系统 mc_rtc。
set(MC_RTC_HONOR_INSTALL_PREFIX ON)
find_package(mc_rtc REQUIRED)
find_package(eigen-quadprog REQUIRED)

add_subdirectory(src)
add_subdirectory(plugin)
add_subdirectory(cmake)

install(FILES include/mc_tasks/ForceConstrainedTransformTask.h DESTINATION include/mc_tasks)
