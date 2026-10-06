import rclpy
from geometry_msgs.msg import Twist
from m20_msgs.msg import RobotMode
from m20_locomotion_bridge.bridge_node import LocomotionBridge


class SpySDK:
    def __init__(self): self.vel = []; self.stops = 0
    def connect(self): return True
    def send_velocity(self, vx, vy, wz): self.vel.append((vx, vy, wz))
    def request_gait(self, g): return True
    def stop(self): self.stops += 1


def arm(bridge):
    m = RobotMode(); m.armed = True; m.mode = RobotMode.MODE_TELEOP
    rclpy.deliver("/m20/mode", m)

def test_ignores_cmd_when_disarmed():
    b = LocomotionBridge(); b.sdk = SpySDK()
    rclpy.deliver("/cmd_vel", Twist())
    assert b.sdk.vel == []

def test_forwards_cmd_when_armed():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    t = Twist(); t.linear.x = 1.0
    rclpy.deliver("/cmd_vel", t)
    assert b.sdk.vel[-1][0] == 1.0

def test_watchdog_halts_on_timeout():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.set_time(10.0)
    rclpy.deliver("/cmd_vel", Twist())
    rclpy.set_time(11.1)          # > 1.0 s default timeout
    b.watchdog()
    assert b.stopped and b.sdk.stops >= 1
    assert b.sdk.vel[-1] == (0.0, 0.0, 0.0)

def test_watchdog_quiet_within_timeout():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.set_time(10.0)
    rclpy.deliver("/cmd_vel", Twist())
    rclpy.set_time(10.3)
    b.watchdog()
    assert not b.stopped

def test_disarm_halts_immediately():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.deliver("/cmd_vel", Twist())
    m = RobotMode(); m.armed = False
    rclpy.deliver("/m20/mode", m)
    assert b.stopped and b.sdk.stops >= 1
