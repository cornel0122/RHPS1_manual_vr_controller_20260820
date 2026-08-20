# This represents a device state of a VR Controller.

string      serial_number
bool        pose_is_valid
float32[16] pose_matrix
uint32      un_packet_num
uint64      ul_button_pressed
uint64      ul_button_touched
float32     trackpad_x
float32     trackpad_y
float32     trigger
bool        menu_button
bool        trackpad_pressed
bool        trackpad_touched
bool        grip_button
