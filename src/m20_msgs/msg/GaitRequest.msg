# Version: 0.1
# A REQUEST for a locomotion configuration. The bridge/vendor controller may reject.
uint8 GAIT_WHEELED=0
uint8 GAIT_LEGGED=1
uint8 GAIT_HYBRID=2
uint8 GAIT_STAND=3
uint8 GAIT_SIT=4

std_msgs/Header header
uint8 gait
string requester          # node name, for audit trail
