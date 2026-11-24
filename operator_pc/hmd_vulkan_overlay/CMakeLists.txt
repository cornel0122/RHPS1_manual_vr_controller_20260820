cmake_minimum_required(VERSION 3.16)
project(head_camera_openvr_texture_overlay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(GLEW REQUIRED glew)

find_library(OPENGL_LIB GL REQUIRED)
find_library(VULKAN_LIB vulkan REQUIRED)
set(OPENVR_ROOT "$ENV{HOME}/openvr" CACHE PATH "OpenVR SDK checkout (headers/ and bin/linux64/)")
find_library(OPENVR_LIB openvr_api REQUIRED PATHS ${OPENVR_ROOT}/bin/linux64 NO_DEFAULT_PATH)

add_executable(head_camera_openvr_texture_overlay src/head_camera_openvr_texture_overlay.cpp)
target_include_directories(head_camera_openvr_texture_overlay PRIVATE
  ${OPENVR_ROOT}/headers
  ${SDL2_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
)
target_link_libraries(head_camera_openvr_texture_overlay
  ${OPENVR_LIB}
  ${OPENGL_LIB}
  ${SDL2_LIBRARIES}
  ${GLEW_LIBRARIES}
)
ament_target_dependencies(head_camera_openvr_texture_overlay rclcpp sensor_msgs)

add_executable(head_camera_openvr_vulkan_overlay src/head_camera_openvr_vulkan_overlay.cpp)
target_include_directories(head_camera_openvr_vulkan_overlay PRIVATE
  ${OPENVR_ROOT}/headers
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(head_camera_openvr_vulkan_overlay
  ${OPENVR_LIB}
  ${VULKAN_LIB}
  ${OpenCV_LIBS}
)
ament_target_dependencies(head_camera_openvr_vulkan_overlay rclcpp sensor_msgs)
