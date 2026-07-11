"""Python mirrors of m20_msgs interfaces for ROS-free unit tests.
Keep constants in sync with src/m20_msgs/msg/*.msg (contracts v0.1)."""

class _Header:
    def __init__(self): self.stamp = 0

class RobotMode:
    MODE_IDLE=0; MODE_TELEOP=1; MODE_ASSISTED=2; MODE_AUTONOMOUS=3; MODE_ESTOP=4
    def __init__(self):
        self.header=_Header(); self.mode=0; self.authority_source=""; self.armed=False

class GaitRequest:
    GAIT_WHEELED=0; GAIT_LEGGED=1; GAIT_HYBRID=2; GAIT_STAND=3; GAIT_SIT=4
    def __init__(self):
        self.header=_Header(); self.gait=0; self.requester=""

class FailsafeStatus:
    FS_NONE=0; FS_BATTERY_LOW=1; FS_BATTERY_CRITICAL=2; FS_COMMS_LOSS=4
    FS_ESTIMATOR_DIVERGED=8; FS_ESTOP=16; FS_MODULE_UNHEALTHY=32; FS_CMD_TIMEOUT=64
    def __init__(self):
        self.header=_Header(); self.active=0; self.severity=0; self.detail=""

class HealthReport:
    STATUS_OK=0; STATUS_WARN=1; STATUS_ERROR=2; STATUS_STALE=3
    def __init__(self):
        self.header=_Header(); self.node_name=""; self.status=0; self.message=""

class MissionItem:
    TYPE_GOTO=0; TYPE_WAIT=1; TYPE_SET_GAIT=2; TYPE_INSPECT=3
    def __init__(self):
        self.type=0; self.goal=None; self.wait_seconds=0.0; self.gait=0; self.label=""

class BehaviorRequest:
    def __init__(self):
        self.header=_Header(); self.behavior_name=""; self.requester=""; self.params_json=""

class BehaviorStatus:
    STATE_IDLE=0; STATE_PRECHECK=1; STATE_RUNNING=2; STATE_SUCCEEDED=3
    STATE_ABORTED=4; STATE_FAILED=5
    def __init__(self):
        self.header=_Header(); self.behavior_name=""; self.state=0
        self.progress=0.0; self.detail=""

class ControlAuthority:
    TIER_VENDOR=0; TIER_POLICY=1
    def __init__(self):
        self.header=_Header(); self.tier=0; self.holder=""; self.lease_ms=0
