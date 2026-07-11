# Version: 0.1
# Published by m20_commander ONLY. Single source of truth for robot mode.
uint8 MODE_IDLE=0
uint8 MODE_TELEOP=1
uint8 MODE_ASSISTED=2
uint8 MODE_AUTONOMOUS=3
uint8 MODE_ESTOP=4

std_msgs/Header header
uint8 mode
string authority_source   # node name that requested the current mode
bool armed
