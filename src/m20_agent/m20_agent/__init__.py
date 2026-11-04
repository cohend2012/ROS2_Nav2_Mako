"""m20_agent: M20-Brain L2 (landmark memory) + L3 (VLM mission agent).

docs/AI_NAV_BRIEF.md §4.2-4.3. The agent PLANS in language and EXECUTES only
through /m20/mission/run (m20_missions) — it can never publish cmd_vel or change
modes (ADR-005). Everything except agent_node.py is ROS-free.
"""
