import rclpy
from m20_msgs.msg import BehaviorRequest, BehaviorStatus, RobotMode, FailsafeStatus
from m20_behaviors.engine import BehaviorEngine


def make_engine_armed():
    e = BehaviorEngine()
    m = RobotMode(); m.armed = True; m.mode = RobotMode.MODE_AUTONOMOUS
    rclpy.deliver("/m20/mode", m)
    return e

def req(name, params=""):
    r = BehaviorRequest(); r.behavior_name = name; r.params_json = params
    rclpy.deliver("/m20/behavior/request", r)

def last_status():
    return rclpy.PUBLISHED["/m20/behavior/status"][-1]


def test_discovers_all_four_behaviors():
    e = BehaviorEngine()
    assert {"camera_scan", "self_right", "jump_pipe", "three_wheel"} <= set(e.registry)

def test_rejects_unknown_behavior():
    e = make_engine_armed()
    req("backflip")
    s = last_status()
    assert s.state == BehaviorStatus.STATE_FAILED and "unknown" in s.detail

def test_rejects_when_disarmed():
    e = BehaviorEngine()
    req("camera_scan", '{"target_class": "valve"}')
    assert last_status().state == BehaviorStatus.STATE_FAILED

def test_precheck_failure_blocks_start():
    e = make_engine_armed()
    req("three_wheel", "{}")   # precheck always fails until Phase 8 (camera_scan now defaults its target)
    s = last_status()
    assert s.state == BehaviorStatus.STATE_FAILED and "precheck" in s.detail
    assert e.active is None

def test_vendor_tier_behavior_runs():
    e = make_engine_armed()
    req("camera_scan", '{"target_class": "valve"}')
    assert e.active is not None and last_status().state == BehaviorStatus.STATE_RUNNING

def test_policy_tier_denied_without_lease():
    e = make_engine_armed()
    req("self_right")   # precheck stub fails first; also lease unavailable in shim
    assert e.active is None and last_status().state == BehaviorStatus.STATE_FAILED

def test_hard_timeout_aborts():
    e = make_engine_armed()
    req("camera_scan", '{"target_class": "valve"}')
    e.active_elapsed = e.active.SPEC.max_duration_s + 1.0
    e.tick()
    assert e.active is None and last_status().state == BehaviorStatus.STATE_ABORTED

def test_mode_drop_aborts_running_behavior():
    e = make_engine_armed()
    req("camera_scan", '{"target_class": "valve"}')
    m = RobotMode(); m.armed = False; m.mode = RobotMode.MODE_IDLE
    rclpy.deliver("/m20/mode", m)
    assert e.active is None and last_status().state == BehaviorStatus.STATE_ABORTED

def test_busy_engine_rejects_second_behavior():
    e = make_engine_armed()
    req("camera_scan", '{"target_class": "valve"}')
    req("camera_scan", '{"target_class": "door"}')
    assert "busy" in last_status().detail
