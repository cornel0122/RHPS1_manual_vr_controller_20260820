set(AUTOLOAD_JointTrackingPlotterPlugin_PLUGIN
    false
    CACHE BOOL "When true the plugin will always be loaded")

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/etc/JointTrackingPlotterPlugin.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/JointTrackingPlotterPlugin.cpp)

set(plugin_HDR src/JointTrackingPlotterPlugin.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(JointTrackingPlotterPlugin "${plugin_SRC}" "${plugin_HDR}")
