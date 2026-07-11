import rclpy
from rclpy import PUBLISHED, SUBS, SERVICES, TIMERS, _Clock, _Logger

class _Param:
    def __init__(self, value): self.value = value

class _Pub:
    def __init__(self, topic): self.topic = topic
    def publish(self, msg): PUBLISHED.setdefault(self.topic, []).append(msg)

class _Future:
    def done(self): return False
    def result(self): return None

class _Client:
    def service_is_ready(self): return False
    def call_async(self, req): return _Future()

class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
    def get_name(self): return self._name
    def get_logger(self): return _Logger()
    def get_clock(self): return _Clock()
    def declare_parameter(self, name, default): self._params[name] = default
    def get_parameter(self, name): return _Param(self._params[name])
    def create_publisher(self, mtype, topic, qos): return _Pub(topic)
    def create_subscription(self, mtype, topic, cb, qos):
        SUBS.setdefault(topic, []).append(cb); return object()
    def create_service(self, stype, name, cb): SERVICES[name] = cb; return object()
    def create_client(self, stype, name): return _Client()
    def create_timer(self, period, cb): TIMERS.append((period, cb)); return object()
