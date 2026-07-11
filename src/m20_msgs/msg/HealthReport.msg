# Version: 0.1
# Per-module heartbeat. Every m20_* node publishes at 1 Hz on /m20/health.
uint8 STATUS_OK=0
uint8 STATUS_WARN=1
uint8 STATUS_ERROR=2
uint8 STATUS_STALE=3

std_msgs/Header header
string node_name
uint8 status
string message
