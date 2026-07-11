"""Minimal rclpy shim for ROS-free unit tests of node logic.
NOT a simulator: no executor, no DDS. Tests drive callbacks/timers directly."""
FAKE_TIME_NS = [0]
PUBLISHED = {}   # topic -> list of msgs
SUBS = {}        # topic -> list of callbacks
SERVICES = {}    # name -> callback
TIMERS = []      # (period_s, callback)

def reset():
    FAKE_TIME_NS[0] = 0
    PUBLISHED.clear(); SUBS.clear(); SERVICES.clear(); TIMERS.clear()

def set_time(seconds: float):
    FAKE_TIME_NS[0] = int(seconds * 1e9)

def advance(seconds: float):
    FAKE_TIME_NS[0] += int(seconds * 1e9)

def deliver(topic, msg):
    for cb in SUBS.get(topic, []):
        cb(msg)

def call_service(name, req, res):
    return SERVICES[name](req, res)

def init(args=None): pass
def shutdown(): pass
def spin(node): raise RuntimeError("shim: don't spin in unit tests")
def create_node(name): from rclpy.node import Node; return Node(name)
def spin_until_future_complete(node, future, timeout_sec=None): pass

class _Time:
    def __init__(self, ns): self.nanoseconds = ns
    def __sub__(self, other): return _Time(self.nanoseconds - other.nanoseconds)
    def to_msg(self): return self.nanoseconds

class _Clock:
    def now(self): return _Time(FAKE_TIME_NS[0])

class _Logger:
    def info(self, m): pass
    def warn(self, m): pass
    def error(self, m): pass
