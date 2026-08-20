add_fsm_state_simple(RelativeRetargetting)
target_link_libraries(RelativeRetargetting
                      PUBLIC mc_neuron_mocap_plugin::mc_neuron_mocap_properties)
add_fsm_state_simple(OrientationRetargetting)
target_link_libraries(OrientationRetargetting
                      PUBLIC mc_neuron_mocap_plugin::mc_neuron_mocap_properties)
add_fsm_state_simple(CoMRetargetting)
add_fsm_state_simple(UpdatePosture)
add_fsm_state_simple(ViveHandBridge)
target_link_libraries(
  ViveHandBridge
  PUBLIC rclcpp::rclcpp
         vr_interface_msgs::vr_interface_msgs__rosidl_typesupport_cpp
         ForceConstrainedTransformTask::ForceConstrainedTransformTask)
add_fsm_state_simple(ViveHeadBridge)
target_link_libraries(
  ViveHeadBridge
  PUBLIC rclcpp::rclcpp
         vr_interface_msgs::vr_interface_msgs__rosidl_typesupport_cpp)
add_fsm_state_simple(IntentRecognitionBridge)
target_link_libraries(
  IntentRecognitionBridge
  PUBLIC rclcpp::rclcpp
         geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
         std_msgs::std_msgs__rosidl_typesupport_cpp)
add_fsm_state_simple(RetargetingControl)
add_fsm_state_simple(RetargetingReference)
add_fsm_state_simple(GripperHandling)
add_fsm_state_simple(FacialExpression)
add_fsm_state_simple(ROSInputState)
add_fsm_state_simple(ROSPublisherState)
add_fsm_state_simple(TriggerTransition)
add_fsm_state_simple(TriggerWalking)
add_fsm_state_simple(MeasureObjectWeight)
add_fsm_state_simple(SynchroWalk)
target_link_libraries(SynchroWalk
                      PUBLIC mc_neuron_mocap_plugin::mc_neuron_mocap_properties)
add_fsm_state_simple(Initial_ISMPC)
add_fsm_state_simple(SharedAutonomy)
target_link_libraries(
  SharedAutonomy
  PUBLIC ForceConstrainedTransformTask::ForceConstrainedTransformTask)
add_fsm_state_simple(UI_SharedAutonomy)
target_link_libraries(
  UI_SharedAutonomy
  PUBLIC rclcpp::rclcpp std_msgs::std_msgs__rosidl_typesupport_cpp)

add_fsm_data_directory(data)
target_link_libraries(
  RelativeRetargetting
  PUBLIC ForceConstrainedTransformTask::ForceConstrainedTransformTask)

set(JoystickControl_SRC
    JoystickControl.cpp
    JoystickControl/ManualSpeedControl.cpp
    JoystickControl/NativeJoystickControl.cpp
    JoystickControl/ROSJoystickControl.cpp
    JoystickControl/ROSJoyMsgControl.cpp
    JoystickControl/UnityHeadJoystickControl.cpp
    JoystickControl/UnityJoystickControl.cpp)
set(JoystickControl_HDR JoystickControl.h JoystickControl/common.h
                        JoystickControl/Modality.h)
add_fsm_state(JoystickControl ${JoystickControl_SRC} ${JoystickControl_HDR})
