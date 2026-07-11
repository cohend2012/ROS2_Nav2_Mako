import rclpy
from m20_msgs.msg import RobotMode, HealthReport, ControlAuthority
from m20_msgs.srv import SetMode, SetControlAuthority
from m20_commander.commander_node import Commander


def set_mode(mode, requester="test"):
    req = SetMode.Request(); req.mode = mode; req.requester = requester
    return rclpy.call_service("/m20/set_mode", req, SetMode.Response())

def set_auth(tier, lease_ms=5000, requester="test"):
    req = SetControlAuthority.Request()
    req.tier = tier; req.lease_ms = lease_ms; req.requester = requester
    return rclpy.call_service("/m20/set_control_authority", req,
                              SetControlAuthority.Response())


def test_starts_idle_disarmed():
    c = Commander()
    assert c.mode == RobotMode.MODE_IDLE and not c.armed
    assert rclpy.PUBLISHED["/m20/mode"][-1].mode == RobotMode.MODE_IDLE

def test_legal_transition_arms():
    c = Commander()
    res = set_mode(RobotMode.MODE_TELEOP)
    assert res.accepted and c.armed and c.mode == RobotMode.MODE_TELEOP

def test_illegal_transition_rejected():
    c = Commander()
    res = set_mode(RobotMode.MODE_AUTONOMOUS)  # IDLE -> AUTONOMOUS is legal
    assert res.accepted
    res = set_mode(RobotMode.MODE_TELEOP)      # AUTONOMOUS -> TELEOP is not
    assert not res.accepted and "illegal" in res.reason

def test_unhealthy_module_forces_estop():
    c = Commander()
    set_mode(RobotMode.MODE_AUTONOMOUS)
    h = HealthReport(); h.node_name = "m20_perception"; h.status = HealthReport.STATUS_ERROR
    rclpy.deliver("/m20/health", h)
    assert c.mode == RobotMode.MODE_ESTOP and not c.armed

def test_mode_change_denied_during_failsafe():
    c = Commander()
    h = HealthReport(); h.status = HealthReport.STATUS_ERROR
    rclpy.deliver("/m20/health", h)
    res = set_mode(RobotMode.MODE_AUTONOMOUS)
    assert not res.accepted and "failsafe" in res.reason

def test_estop_only_exits_to_idle():
    c = Commander()
    set_mode(RobotMode.MODE_ESTOP)
    assert not set_mode(RobotMode.MODE_TELEOP).accepted
    assert set_mode(RobotMode.MODE_IDLE).accepted

def test_policy_lease_denied_when_disarmed():
    c = Commander()
    res = set_auth(ControlAuthority.TIER_POLICY)
    assert not res.granted

def test_policy_lease_granted_and_expires():
    c = Commander()
    set_mode(RobotMode.MODE_AUTONOMOUS)
    res = set_auth(ControlAuthority.TIER_POLICY, lease_ms=2000)
    assert res.granted and c.authority_tier == ControlAuthority.TIER_POLICY
    rclpy.advance(3.0)
    c.tick()
    assert c.authority_tier == ControlAuthority.TIER_VENDOR

def test_failsafe_revokes_lease():
    c = Commander()
    set_mode(RobotMode.MODE_AUTONOMOUS)
    assert set_auth(ControlAuthority.TIER_POLICY).granted
    h = HealthReport(); h.status = HealthReport.STATUS_ERROR
    rclpy.deliver("/m20/health", h)
    assert c.authority_tier == ControlAuthority.TIER_VENDOR

def test_second_lease_denied_while_held():
    c = Commander()
    set_mode(RobotMode.MODE_AUTONOMOUS)
    assert set_auth(ControlAuthority.TIER_POLICY, requester="a").granted
    assert not set_auth(ControlAuthority.TIER_POLICY, requester="b").granted
