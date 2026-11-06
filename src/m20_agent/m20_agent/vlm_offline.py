"""Offline backend: no model, no network. Used by CI and as the field fallback when
the link to the VLM drops (docs/AI_NAV_BRIEF.md §4.3).

Planning: keyword rules over the operator request + registry. It understands
  "read all (the) gauges" / "check every gauge"   -> read_gauge for each gauge, nearest-first
  "<gauge name|alias>" (unless "go to/drive to")    -> read_gauge
  "go to / visit / inspect <landmark>"             -> goto / inspect
  "patrol" / "tour"                                -> goto every non-gauge landmark
  "find / search for / look for / locate gauges"   -> find_gauges (no prior positions;
                                                      also when gauges are asked for and
                                                      none are on the map yet)
  "...and return / come back / where you started"  -> trailing return_home
Anything it cannot match becomes a 1 s wait plus a rationale. It never guesses a target.

Gauge reading: classical needle fit (gauge_dial.read_dial) on the camera frame.
"""
import io
import math
import re

import numpy as np

from m20_agent.gauge_dial import read_dial

UNIT_CASE = {"degc": "degC", "degf": "degF", "kpa": "kPa", "mpa": "MPa", "°c": "°C", "°f": "°F",
             "m3/h": "m3/h", "inh2o": "inH2O"}
STOP = {"the", "a", "an", "read", "what", "does", "say", "says", "gauge", "gauges", "on", "at",
        "of", "is", "me", "tell", "check", "please", "and", "come", "back", "to", "dial", "value",
        "reading", "go", "oil"}
HOME_WORDS = ("return", "come back", "where you started", "go home", "back home",
              "back to start", "return home")


class OfflineBackend:
    name = "offline"

    def __init__(self, registry):
        self.reg = registry            # needs the live registry (not just its text)

    TEACH = re.compile(r"(?:remember|save|mark|store)\s+(?:this|here|this\s+(?:place|spot|location|room))\s+"
                       r"as\s+(?P<a>.+)|^(?:this|here)\s+is\s+(?P<b>.+)|^call\s+(?:this|here)\s+(?P<c>.+)")
    SCALE = re.compile(r"(?:(?:set\s+)?(?:the\s+)?scale\s+(?:of|on|for)\s+)?(?P<t>[a-z]+-\d+|found_gauge_\d+)"
                       r"\s+(?:is|reads|goes|runs|scale is|to)?\s*(?:from\s+)?(?P<lo>-?\d+(?:\.\d+)?)"
                       r"\s*(?:to|-|–)\s*(?P<hi>-?\d+(?:\.\d+)?)\s*(?P<u>[a-z°%/0-9]+)")
    ROUTINE = re.compile(r"\b(?:start|run|begin|do|execute|launch)\s+(?:the\s+|your\s+|my\s+)?"
                         r"(?P<r>mission|routine|rounds?|patrol|[a-z0-9 _-]*?(?:round|routine|mission|patrol|tour))\b")

    def plan(self, request, registry_text=None):
        q = request.lower().strip().rstrip(".!?")
        steps, notes = [], []
        m = self.TEACH.search(q)
        if m:
            name = (m.group("a") or m.group("b") or m.group("c")).strip()
            return {"steps": [{"action": "save_place", "target": name}],
                    "rationale": f"offline: teach current pose as {name!r}"}
        if self.reg.routines and re.search(r"\b(start|run|begin|do|execute|launch)\b", q):
            keys = sorted(((k.lower(), n) for n, r in self.reg.routines.items()
                           for k in [n, n.replace("_", " ")] + r["aliases"]), key=lambda kv: -len(kv[0]))
            for k, n in keys:
                if re.search(r"\b" + re.escape(k) + r"\b", q):
                    return {"steps": [{"action": "run_routine", "target": n}],
                            "rationale": f"offline: routine {n}"}
        m = self.ROUTINE.search(q)
        if m and self.reg.routines:
            word = m.group("r").strip()
            name = self.reg.resolve_routine(word) or self.reg.resolve_routine(
                word.replace("the ", "")) or (self.reg.default_routine if word in
                                              ("mission", "routine", "round", "rounds", "patrol") else None)
            if name:
                return {"steps": [{"action": "run_routine", "target": name}],
                        "rationale": f"offline: routine {name}"}
            notes.append(f"no routine called {word!r}")
        for m in list(self.SCALE.finditer(q)):           # "DIAL-004 is 0 to 100 psi, ..."
            steps.append({"action": "set_gauge_scale", "target": m.group("t").strip(),
                          "min": float(m.group("lo")), "max": float(m.group("hi")),
                          "units": UNIT_CASE.get(m.group("u"), m.group("u"))})
        if steps:
            q = self.SCALE.sub(" ", q)
            if not re.search(r"\b(read|check|go|find|return|come back)\b", q):
                return {"steps": steps, "rationale": "offline: gauge scale(s)"}
        gauges = [lm.name for lm in self.reg.of_type("gauge")]
        find = re.search(r"\b(find|search|look for|locate|discover|hunt for|map)\b.*\b(gauges?|dials?|meters?)\b", q)
        if find or (not gauges and re.search(r"\bgauges\b|\b(all|every|each)\b.*\bgauge\b", q)):
            steps.append({"action": "find_gauges"})
            if not find:
                notes.append("no gauges on the map yet: searching for them")
        elif re.search(r"\b(all|every|each)\b.*\bgauges?\b|\bgauges\b", q):
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
            gauge_word = re.search(r"\b(gauges?|dials?|meters?|reading|pressure|temperature)\b", q)
            driving = re.search(r"\b(go to|drive to|move to)\b", q)
            for k, (pos, name) in enumerate(sorted(seen)):
                lm = self.reg.get(name)
                if lm.type != "gauge" and gauge_word and not driving:
                    # "the gauge ON the pipe rack": the gauge nearest that landmark
                    g = min(self.reg.of_type("gauge"), default=None,
                            key=lambda g: math.hypot(g.position[0] - lm.position[0],
                                                     g.position[1] - lm.position[1]))
                    if g is not None and math.hypot(g.position[0] - lm.position[0],
                                                    g.position[1] - lm.position[1]) < 3.0:
                        if g.name not in [st.get("target") for st in steps]:
                            steps.append({"action": "read_gauge", "target": g.name})
                        notes.append(f"the gauge at {lm.name} is {g.name}")
                        continue
                if lm.type == "gauge" and not driving:
                    steps.append({"action": "read_gauge", "target": name})
                elif re.search(r"\b(inspect|look at|check)\b", q):
                    steps.append({"action": "inspect", "target": name,
                                  "question": "Describe the condition of this equipment."})
                else:
                    steps.append({"action": "goto", "target": name})
            if not steps and gauge_word:
                # described, not named ("the oil pipe gauge"): best word overlap with a
                # gauge's name/aliases, only if ONE gauge wins outright
                words = set(re.findall(r"[a-z0-9]+", q)) - STOP
                def score(g):
                    keys = set(re.findall(r"[a-z0-9]+", " ".join([g.name] + list(g.aliases)).lower())) - STOP
                    return sum(1 for w in words for k in keys if w == k or (len(w) > 3 and (w.startswith(k) or k.startswith(w))))
                ranked = sorted(self.reg.of_type("gauge"), key=score, reverse=True)
                if ranked and score(ranked[0]) > 0 and (len(ranked) == 1 or score(ranked[0]) > score(ranked[1])):
                    steps.append({"action": "read_gauge", "target": ranked[0].name})
                    notes.append(f"'{q}' best matches {ranked[0].name} ({', '.join(ranked[0].aliases[:2])})")
                elif ranked and score(ranked[0]) > 0:
                    tied = [g.name for g in ranked if score(g) == score(ranked[0])]
                    return {"steps": [{"action": "wait", "seconds": 1}],
                            "rationale": f"Which gauge do you mean: {', '.join(tied)}?"}
            if not steps and re.search(r"\b(patrol|tour|survey)\b", q):
                for n in self.reg.nearest_order(
                        [lm.name for lm in (self.reg.get(x) for x in self.reg.names())
                         if lm.type not in ("gauge", "home")], (0.0, 0.0)):
                    steps.append({"action": "goto", "target": n})
        # a destination the registry doesn't know: say so instead of doing only the
        # part we understood (going straight "home" would look like success)
        for m in re.finditer(r"\b(?:(?:go|drive|head|walk|take me|navigate)\s+(?:back\s+)?to|find|where(?:'s| is))"
                             r"\s+(?:the\s+)?([a-z0-9' _-]+?)(?:\s+and\b|,|\?|$)", q):
            place = m.group(1).strip()
            if re.search(r"\b(gauges?|dials?|meters?)\b", place) or any(
                    st["action"] in ("find_gauges", "read_gauge") for st in steps):
                continue                               # a gauge task, not a place
            if place in ("home", "start", "base", "where you started", "the start") or place.startswith("where"):
                continue
            if self.reg.resolve(place) is None and self.reg.resolve(place.replace(" ", "_")) is None:
                return {"steps": [{"action": "wait", "seconds": 1}],
                        "rationale": f"I don't know where '{place}' is. Drive me there and say "
                                     f"'this is the {place}' to teach me."}
        if any(w in q for w in HOME_WORDS):
            steps.append({"action": "return_home"})
        if not steps and re.search(r"\b(gauge|dial|meter)\b", q) and not gauges:
            return {"steps": [{"action": "wait", "seconds": 1}],
                    "rationale": "No gauges are on this map yet. Say 'find the gauges' and I'll "
                                 "search for them first."}
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
