"""Behavior plugin contract for the M20 Behavior Engine.

Every behavior — including ones nobody has thought of yet — implements this
interface. The engine (engine.py) owns the lifecycle; behaviors own only their
own logic. That separation is what makes "novel items we have not thought of"
cheap to add later: a new behavior is a new file in behaviors/, nothing else.

Lifecycle:  PRECHECK -> RUNNING -> (SUCCEEDED | FAILED | ABORTED)

Two execution tiers (see ADR-008 in docs/SECOND_BRAIN.md):
  TIER_VENDOR  — behavior composes high-level commands (cmd_vel, gait requests,
                 body-pose commands). Example: camera_scan.
  TIER_POLICY  — behavior needs joint-level control; the engine must first obtain
                 a control-authority lease from Commander, then stream a trained
                 policy (ONNX from Deep Robotics rl_training / Isaac Lab) through
                 the bridge's low-level channel. Examples: jump_pipe, self_right,
                 three_wheel. Lease not granted -> behavior never starts.

Safety obligations of every behavior:
  * precheck() must verify EVERYTHING it assumes (battery margin, flat ground,
    clearance, estimator health). Optimistic prechecks are how robots break.
  * abort() must leave the robot in a state the vendor controller can recover
    (standing or safely settled). It can be called at ANY time, including by
    Commander revoking authority mid-jump — plan the landing first.
  * step() runs at the engine tick rate and must never block.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum


class Tier(IntEnum):
    VENDOR = 0   # high-level command composition
    POLICY = 1   # leased joint-level control (offboard analog)


class Result(IntEnum):
    RUNNING = 0
    SUCCEEDED = 1
    FAILED = 2


@dataclass
class BehaviorSpec:
    name: str
    tier: Tier
    description: str
    max_duration_s: float          # engine hard-aborts past this
    requires_estimator: bool = True
    min_battery_pct: float = 30.0  # dynamic behaviors are power-hungry
    tags: list = field(default_factory=list)


class Behavior(ABC):
    """Subclass, set SPEC, implement the four methods, drop the file in
    behaviors/. The engine auto-discovers it at startup."""

    SPEC: BehaviorSpec = None

    def __init__(self, node):
        self.node = node  # rclpy node: use for pubs/subs/clocks/logging

    @abstractmethod
    def precheck(self, params: dict) -> tuple[bool, str]:
        """Return (ok, reason). Called once before any motion."""

    @abstractmethod
    def start(self, params: dict) -> None:
        """Called once after precheck passes (and, for TIER_POLICY, after the
        authority lease is granted)."""

    @abstractmethod
    def step(self, dt: float) -> Result:
        """Called at engine rate while RUNNING. Non-blocking."""

    @abstractmethod
    def abort(self) -> None:
        """Emergency wind-down to a vendor-recoverable state. Always callable."""

    def progress(self) -> float:
        return 0.0
