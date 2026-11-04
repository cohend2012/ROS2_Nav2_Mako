"""Offline backend: no model, no network. Used by CI and as the field fallback when
the link to the VLM drops (docs/AI_NAV_BRIEF.md §4.3).

Planning: keyword rules over the operator request + registry. It understands
  "read all (the) gauges" / "check every gauge"   -> read_gauge for each gauge, nearest-first
  "<gauge name|alias>" (unless "go to/drive to")    -> read_gauge
  "go to / visit / inspect <landmark>"             -> goto / inspect
  "patrol" / "tour"                                -> goto every non-gauge landmark
  "...and return / come back / where you started"  -> trailing return_home
Anything it cannot match becomes a 1 s wait plus a rationale. It never guesses a target.

Gauge reading: classical needle fit (gauge_dial.read_dial) on the camera frame.
"""
import io
import re

import numpy as np

from m20_agent.gauge_dial import read_dial

HOME_WORDS = ("return", "come back", "where you started", "go home", "back home",
              "back to start", "return home")


class OfflineBackend:
    name = "offline"

    def __init__(self, registry):
        self.reg = registry            # needs the live registry (not just its text)

    def plan(self, request, registry_text=None):
        q = request.lower()
        steps, notes = [], []
        gauges = [lm.name for lm in self.reg.of_type("gauge")]
        if re.search(r"\b(all|every|each)\b.*\bgauges?\b|\bgauges\b", q):
            for g in self.reg.nearest_order(gauges, (0.0, 0.0)):
                steps.append({"action": "read_gauge", "target": g})
        else:
            # longest names/aliases first so "tank 1 gauge" beats "tank 1"
            cands = []
            for lm in (self.reg.get(n) for n in self.reg.names()):
                if lm.type == "home":
                    continue
                for key in [lm.name, lm.name.replace("_", " ")] + list(lm.aliases):
                    cands.append((key.lower(), lm))
            cands.sort(key=lambda kv: -len(kv[0]))
            seen = []
            text = q
            for key, lm in cands:
                pos = text.find(key)
                if pos >= 0 and lm.name not in [s for _, s in seen]:
                    seen.append((pos, lm.name))
                    text = text[:pos] + "#" * len(key) + text[pos + len(key):]
            for _, name in sorted(seen):
                lm = self.reg.get(name)
                if lm.type == "gauge" and not re.search(r"\b(go to|drive to|move to)\b", q):
                    steps.append({"action": "read_gauge", "target": name})
                elif re.search(r"\b(inspect|look at|check)\b", q):
                    steps.append({"action": "inspect", "target": name,
                                  "question": "Describe the condition of this equipment."})
                else:
                    steps.append({"action": "goto", "target": name})
            if not steps and re.search(r"\b(patrol|tour|survey)\b", q):
                for n in self.reg.nearest_order(
                        [lm.name for lm in (self.reg.get(x) for x in self.reg.names())
                         if lm.type not in ("gauge", "home")], (0.0, 0.0)):
                    steps.append({"action": "goto", "target": n})
        if any(w in q for w in HOME_WORDS):
            steps.append({"action": "return_home"})
        if not steps:
            notes.append("offline planner found no known landmark or task in the request")
            steps = [{"action": "wait", "seconds": 1}]
        return {"steps": steps, "rationale": "; ".join(notes) or "offline keyword plan"}

    def read_gauge(self, png, name, spec):
        from PIL import Image
        rgb = np.asarray(Image.open(io.BytesIO(png)).convert("RGB"))
        frac, conf, detail = read_dial(rgb)
        if frac is None:
            return {"readable": False, "value": 0.0, "units": spec["units"],
                    "confidence": 0.0, "notes": detail}
        v = spec["min"] + frac * (spec["max"] - spec["min"])
        return {"readable": True, "value": round(float(v), 2), "units": spec["units"],
                "confidence": round(conf, 2), "notes": "classical needle fit: " + detail}

    def describe(self, png, question):
        return {"answer": "offline backend: image captured, no description model available",
                "anomaly": False}
