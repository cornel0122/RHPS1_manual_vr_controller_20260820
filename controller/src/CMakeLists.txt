set(controller_SRC
    ANAAvatarController.cpp
    motion_prediction/Algebric_derivation.cpp
    motion_prediction/Kalman_Filter.cpp
    motion_prediction/MinJerk_Trajectory.cpp
    motion_prediction/Motion_Prediction.cpp
    motion_prediction/RLS_Filter.cpp
    joystick/joystick.cc)

set(controller_HDR
    ANAAvatarController.h
    ROSSubscriber.h
    motion_prediction/Algebric_derivation.h
    motion_prediction/Kalman_Filter.h
    motion_prediction/MinJerk_Trajectory.h
    motion_prediction/Motion_Prediction.h
    motion_prediction/RLS_Filter.h
    joystick/joystick.hh)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME}
                      PROPERTIES COMPILE_FLAGS "-DANAAvatarController_EXPORTS")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC mc_rtc::mc_control_fsm
         mc_rtc::mc_rtc_ros
         lipm_walking_controller::lipm_walking_controller)
if(TARGET mc_ana_avatar_controller::ROS)
  target_link_libraries(${PROJECT_NAME} PUBLIC mc_ana_avatar_controller::ROS)
endif()

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME
                                                            "${PROJECT_NAME}")
set_target_properties(
  ${PROJECT_NAME}_controller
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY lib/$<CONFIGURATION>
             LIBRARY_OUTPUT_DIRECTORY lib/$<CONFIGURATION>
             RUNTIME_OUTPUT_DIRECTORY lib/$<CONFIGURATION>)
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
