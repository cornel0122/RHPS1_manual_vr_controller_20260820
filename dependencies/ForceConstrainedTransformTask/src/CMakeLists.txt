add_library(${PROJECT_NAME} SHARED ForceConstrainedTransformTask.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_tasks eigen-quadprog::eigen-quadprog)
target_compile_definitions(${PROJECT_NAME} PRIVATE -D${PROJECT_NAME}_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
