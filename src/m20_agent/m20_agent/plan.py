"""Mission plans: the ONLY thing a VLM is allowed to produce for motion.

A plan is a JSON list of steps:
  {"action": "goto",        "target": "<landmark name>"}
  {"action": "read_gauge",  "target": "<gauge landmark>"}   # goto approach + read
  {"action": "inspect",     "target": "<landmark>", "question": "..."}  # goto + describe
  {"action": "wait",        "seconds": 5}
  {"action": "return_home"}

validate() is the grounding step (SayCan-style: the language model proposes, the
robot's actual capabilities dispose). Unknown targets, reading a non-gauge, or an
over-long plan are REJECTED before anything moves — never "best-effort" executed.
"""
ACTIONS = ("goto", "read_gauge", "inspect", "wait", "return_home")
MAX_STEPS = 40
MAX_WAIT_S = 120.0

PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": list(ACTIONS)},
                    "target": {"type": "string"},
                    "question": {"type": "string"},
                    "seconds": {"type": "number"},
                },
                "required": ["action"],
                "additionalProperties": False,
            },
        },
        "rationale": {"type": "string"},
    },
    "required": ["steps", "rationale"],
    "additionalProperties": False,
}


class PlanError(ValueError):
    pass


def validate(plan, registry):
    """Returns a normalized list of steps (targets resolved to canonical names)."""
    if not isinstance(plan, dict) or not isinstance(plan.get("steps"), list):
        raise PlanError("plan must be an object with a 'steps' list")
    steps = plan["steps"]
    if not steps:
        raise PlanError("empty plan")
    if len(steps) > MAX_STEPS:
        raise PlanError(f"plan has {len(steps)} steps (max {MAX_STEPS})")
    out = []
    for i, s in enumerate(steps):
        a = s.get("action")
        if a not in ACTIONS:
            raise PlanError(f"step {i}: unknown action {a!r}")
        if a == "wait":
            sec = float(s.get("seconds", 0))
            if not 0 < sec <= MAX_WAIT_S:
                raise PlanError(f"step {i}: wait seconds must be in (0, {MAX_WAIT_S}]")
            out.append({"action": "wait", "seconds": sec})
            continue
        if a == "return_home":
            out.append({"action": "return_home"})
            continue
        tgt = s.get("target")
        if not tgt:
            raise PlanError(f"step {i}: {a} needs a target")
        lm = registry.resolve(tgt)
        if lm is None:
            raise PlanError(f"step {i}: unknown landmark {tgt!r} "
                            f"(known: {', '.join(registry.names())})")
        if a == "read_gauge" and lm.type != "gauge":
            raise PlanError(f"step {i}: {lm.name} is a {lm.type}, not a gauge")
        step = {"action": a, "target": lm.name}
        if a == "inspect":
            step["question"] = s.get("question") or "Describe the condition of this equipment."
        out.append(step)
    return out
