"""m20_ai_nav: learned local navigation for the M20 (docs/AI_NAV_BRIEF.md, L1).

The policy's action is a BODY VELOCITY (vx, wz) — never joint commands. Everything
in this package except `sim/` and `controller_node` is ROS-free and MuJoCo-free so
it unit-tests anywhere (`make unit`).
"""
