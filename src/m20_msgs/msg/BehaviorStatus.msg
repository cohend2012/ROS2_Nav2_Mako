# Version: 0.1
# Published by m20_behaviors during behavior lifecycle.
uint8 STATE_IDLE=0
uint8 STATE_PRECHECK=1
uint8 STATE_RUNNING=2
uint8 STATE_SUCCEEDED=3
uint8 STATE_ABORTED=4
uint8 STATE_FAILED=5

std_msgs/Header header
string behavior_name
uint8 state
float32 progress          # 0..1 best-effort
string detail
