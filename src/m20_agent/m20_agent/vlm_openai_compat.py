"""OpenAI-compatible backend: the NVIDIA Cosmos Reason path (docs/AI_NAV_BRIEF.md §4.3).

Talks to any server exposing /v1/chat/completions. Cosmos Reason 2 or the Cosmos 3
Reasoner can be served that way with `vllm serve nvidia/Cosmos-Reason2-8B` or as an
NVIDIA NIM. Configure it with:

  M20_COSMOS_BASE_URL   e.g. http://gpu-box:8000/v1   (required)
  M20_COSMOS_MODEL      e.g. nvidia/Cosmos-Reason2-8B (required)
  M20_COSMOS_API_KEY    only if the endpoint needs one (NIM cloud); default "none"

Reasoning VLMs served this way do not all implement function calling, so this
backend asks for one JSON object in the reply and extracts it. The schema is then
validated by the same plan.validate() as every other backend. Cosmos-Reason models
put their chain of thought in <think>...</think>; that is stripped before parsing.
"""
import base64
import json
import os
import re

from m20_agent import prompts
from m20_agent.vlm_claude import BackendError


def _extract_json(text):
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, flags=re.S)
    cand = m.group(1) if m else None
    if cand is None:
        start = text.find("{")
        end = text.rfind("}")
        if start < 0 or end <= start:
            raise BackendError(f"no JSON object in model reply: {text[:200]!r}")
        cand = text[start:end + 1]
    try:
        return json.loads(cand)
    except json.JSONDecodeError as e:
        raise BackendError(f"invalid JSON from model: {e}: {cand[:200]!r}") from e


DEFAULT_BASE_URL = "https://integrate.api.nvidia.com/v1"
DEFAULT_MODEL = "nvidia/cosmos-reason2-8b"


class OpenAICompatBackend:
    name = "cosmos"

    def __init__(self, base_url=None, model=None, api_key=None, client=None):
        # default: NVIDIA's hosted Cosmos Reason 2 (OpenAI-compatible, key in
        # M20_COSMOS_API_KEY / NVIDIA_API_KEY); point at a local vLLM / NIM to run on-site
        base_url = base_url or os.environ.get("M20_COSMOS_BASE_URL") or DEFAULT_BASE_URL
        self.model = model or os.environ.get("M20_COSMOS_MODEL") or DEFAULT_MODEL
        if client is None and (not base_url or not self.model):
            raise BackendError("cosmos backend needs M20_COSMOS_BASE_URL and M20_COSMOS_MODEL")
        if client is None:
            from openai import OpenAI          # lazy: only needed for this backend
            client = OpenAI(base_url=base_url, timeout=60.0, max_retries=1,
                            api_key=api_key or os.environ.get("M20_COSMOS_API_KEY")
                            or os.environ.get("NVIDIA_API_KEY") or "none")
        self.client = client

    def _chat(self, system, content, max_tokens=4096):
        try:
            r = self.client.chat.completions.create(
                model=self.model, max_tokens=max_tokens,
                messages=[{"role": "system", "content": system},
                          {"role": "user", "content": content}])
        except Exception as e:  # noqa: BLE001 — any transport/server failure is a backend error
            raise BackendError(f"cosmos endpoint error: {e}") from e
        return _extract_json(r.choices[0].message.content or "")

    @staticmethod
    def _img(png):
        return {"type": "image_url", "image_url": {
            "url": "data:image/png;base64," + base64.standard_b64encode(png).decode()}}

    def plan(self, request, registry_text):
        return self._chat(
            prompts.PLANNER_SYSTEM.format(registry=registry_text),
            prompts.PLAN_INSTRUCTION.format(request=request).replace(
                "Call the submit_plan tool with the plan.",
                'Reply with ONLY a JSON object: {"steps": [...], "rationale": "..."}, '
                'each step {"action": ..., "target": ...} (wait uses "seconds").'))

    def read_gauge(self, png, name, spec):
        txt = prompts.gauge_instruction(name, spec).replace(
            "call the report_reading tool.",
            'reply with ONLY a JSON object {"readable": bool, "value": number, '
            '"units": str, "confidence": 0-1, "notes": str}.')
        return self._chat(prompts.GAUGE_SYSTEM, [self._img(png), {"type": "text", "text": txt}])

    def describe(self, png, question):
        return self._chat(prompts.INSPECT_SYSTEM, [
            self._img(png),
            {"type": "text", "text": question + '\nReply with ONLY a JSON object '
                                                '{"answer": str, "anomaly": bool}.'}])

    def describe_asset(self, png, tag):
        """Asset-register context for a found dial (what it is on, what is around it)."""
        keys = ", ".join(f'"{k}"' for k in prompts.ASSET_SCHEMA["properties"])
        out = self._chat(prompts.ASSET_SYSTEM, [
            self._img(png),
            {"type": "text", "text": prompts.ASSET_INSTRUCTION.format(tag=tag) +
             f"\nReply with ONLY a JSON object with keys {keys} "
             '(labels_visible: list of strings; anomaly: bool; confidence: 0-1).'}])
        out["model"] = self.model
        return out

    def plan_route(self, map_png, system, instruction):
        """Global route as waypoints from a map image (m20_agent.vlm_route)."""
        return self._chat(system, [self._img(map_png), {"type": "text", "text": instruction}])

