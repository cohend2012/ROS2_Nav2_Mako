# Version: 0.1
# Bitfield of currently active failsafes. Published by m20_commander at 2 Hz and on change.
uint16 FS_NONE=0
uint16 FS_BATTERY_LOW=1
uint16 FS_BATTERY_CRITICAL=2
uint16 FS_COMMS_LOSS=4
uint16 FS_ESTIMATOR_DIVERGED=8
uint16 FS_ESTOP=16
uint16 FS_MODULE_UNHEALTHY=32
uint16 FS_CMD_TIMEOUT=64

std_msgs/Header header
uint16 active             # OR of FS_* flags
uint8 severity            # 0 none, 1 warn (degrade), 2 critical (halt)
string detail
