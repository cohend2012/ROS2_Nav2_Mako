"""Claude backend (Anthropic SDK). The default backend for development.

Credentials: the SDK's normal resolution (ANTHROPIC_API_KEY, or an `ant auth login`
profile). Model: claude-opus-5-5 unless M20_AGENT_MODEL overrides it.

Each call returns structured data through a strict tool (submit_plan /
report_reading). The prompt names the tool to call; tool_choice stays "auto",
because forced tool choice is rejected on this model. Server-side refusal fallback
is enabled ("default" routing), and stop_reason is checked before content is read.
"""
import base64
import json
import os

import anthropic

from m20_agent import prompts
from m20_agent.plan import PLAN_SCHEMA

DEFAULT_MODEL = "claude-opus-5-5"
FALLBACK_BETA = "server-side-fallback-2026-07-01"

READING_SCHEMA = {
    "type": "object",
    "properties": {
        "readable": {"type": "boolean"},
        "value": {"type": "number"},
        "units": {"type": "string"},
        "confidence": {"type": "number"},
        "notes": {"type": "string"},
    },
    "required": ["readable", "value", "units", "confidence", "notes"],
    "additionalProperties": False,
}


class BackendError(RuntimeError):
    pass


class ClaudeBackend:
    name = "claude"

    def __init__(self, model=None, client=None, effort="medium"):
        self.model = model or os.environ.get("M20_AGENT_MODEL", DEFAULT_MODEL)
        self.client = client or anthropic.Anthropic()
        self.effort = effort

    def _call(self, system, content, tool_name, tool_desc, schema, max_tokens=16000):
        tool = {"name": tool_name, "description": tool_desc, "input_schema": schema,
                "strict": True}
        try:
            r = self.client.beta.messages.create(
                model=self.model, max_tokens=max_tokens, system=system,
                messages=[{"role": "user", "content": content}],
                tools=[tool], tool_choice={"type": "auto"},
                output_config={"effort": self.effort},
                betas=[FALLBACK_BETA], fallbacks="default")
        except anthropic.APIConnectionError as e:
            raise BackendError(f"Claude unreachable: {e}") from e
        except anthropic.RateLimitError as e:
            raise BackendError(f"Claude rate-limited: {e}") from e
        except anthropic.APIStatusError as e:
            raise BackendError(f"Claude API error {e.status_code}: {e.message}") from e
        if r.stop_reason == "refusal":
            cat = getattr(r.stop_details, "category", None) if r.stop_details else None
            raise BackendError(f"Claude declined the request (category {cat})")
        if r.stop_reason == "max_tokens":
            raise BackendError("Claude hit max_tokens before answering")
        for b in r.content:
            if b.type == "tool_use" and b.name == tool_name:
                return b.input if isinstance(b.input, dict) else json.loads(b.input)
        text = " ".join(b.text for b in r.content if b.type == "text")
        raise BackendError(f"Claude did not call {tool_name}: {text[:300]}")

    def plan(self, request, registry_text):
        return self._call(
            prompts.PLANNER_SYSTEM.format(registry=registry_text),
            [{"type": "text", "text": prompts.PLAN_INSTRUCTION.format(request=request)}],
            "submit_plan", "Submit the robot mission plan.", PLAN_SCHEMA)

    def read_gauge(self, png, name, spec):
        img = base64.standard_b64encode(png).decode("utf-8")
        return self._call(
            prompts.GAUGE_SYSTEM,
            [{"type": "image", "source": {"type": "base64", "media_type": "image/png",
                                          "data": img}},
             {"type": "text", "text": prompts.gauge_instruction(name, spec)}],
            "report_reading", "Report the gauge reading.", READING_SCHEMA)

    def describe(self, png, question):
        img = base64.standard_b64encode(png).decode("utf-8")
        schema = {"type": "object", "properties": {"answer": {"type": "string"},
                                                   "anomaly": {"type": "boolean"}},
                  "required": ["answer", "anomaly"], "additionalProperties": False}
        return self._call(
            prompts.INSPECT_SYSTEM,
            [{"type": "image", "source": {"type": "base64", "media_type": "image/png",
                                          "data": img}},
             {"type": "text", "text": f"{question}\n\nCall the report_inspection tool."}],
            "report_inspection", "Report what the image shows.", schema)
