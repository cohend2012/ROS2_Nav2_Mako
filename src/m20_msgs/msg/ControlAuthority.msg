# Version: 0.1
# Which tier currently owns the actuators. Published by m20_commander ONLY.
# Tier changes are requested via SetControlAuthority.srv and can be revoked
# unilaterally by Commander at any time (failsafe -> always TIER_VENDOR + halt).
uint8 TIER_VENDOR=0       # high-level: cmd_vel + gait requests via vendor controller
uint8 TIER_POLICY=1       # low-level: custom policy streams joint targets (offboard analog)

std_msgs/Header header
uint8 tier
string holder             # node holding authority (e.g. "m20_behaviors/jump_pipe")
uint32 lease_ms           # authority auto-reverts if not renewed within this lease
