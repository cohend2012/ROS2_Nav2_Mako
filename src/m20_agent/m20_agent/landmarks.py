"""L2 semantic memory v1: a named landmark registry (map frame).

YAML schema (config/oil_gas_field_landmarks.yaml):
  frame: map
  landmarks:
    - name: gauge_pg101          # unique id the agent must use verbatim
      type: gauge                # gauge | tank | valve | wellhead | skid | area | home
      aliases: [PG-101, "tank 1 pressure gauge"]
      position: [x, y]           # the object itself
      approach: [x, y, yaw]      # where the robot stands to look at it (camera faces yaw)
      gauge: {units: psi, min: 0, max: 100}   # gauges only

  routines:                      # saved missions ("start the mission")
    morning_round:
      aliases: [the mission, morning round, rounds]
      description: read every gauge and come back
      steps: [{action: read_gauge, target: gauge_pg101}, ..., {action: return_home}]
  default_routine: morning_round # what "start the mission" means

TAUGHT places ("this is the kitchen") are written to a separate overlay file
(`taught_path`), never into the curated config: the operator can teach and
re-teach places at runtime, but cannot overwrite a surveyed landmark by voice.

`home` is NOT in the file: the executor records it at mission start (set_home).
Runtime perception adds entries with add(); the registry is the agent's ONLY
vocabulary for motion targets — a name that isn't here cannot be driven to.
"""
import math
import os
from dataclasses import dataclass, field

import yaml

TYPES = ("gauge", "tank", "valve", "wellhead", "skid", "rack", "area", "room", "dock",
         "home")


def _lm_from(d):
    return Landmark(name=d["name"], type=d["type"], position=tuple(d["position"]),
                    approach=tuple(d["approach"]), aliases=d.get("aliases", []),
                    gauge=d.get("gauge"))


@dataclass
class Landmark:
    name: str
    type: str
    position: tuple
    approach: tuple
    aliases: list = field(default_factory=list)
    gauge: dict = None
    taught: bool = False

    def summary(self):
        s = f"{self.name} ({self.type}) at ({self.position[0]:.1f}, {self.position[1]:.1f})"
        if self.aliases:
            s += f" aka {', '.join(self.aliases)}"
        if self.gauge:
            s += f" [{self.gauge['min']}-{self.gauge['max']} {self.gauge['units']}]"
        return s


class LandmarkRegistry:
    def __init__(self, frame="map"):
        self.frame = frame
        self._lm = {}
        self.routines = {}
        self.default_routine = None
        self.taught_path = None

    @classmethod
    def load(cls, path, taught_path=None):
        with open(path) as f:
            doc = yaml.safe_load(f)
        if not doc or "landmarks" not in doc:
            raise ValueError(f"{path}: no 'landmarks' list")
        reg = cls(doc.get("frame", "map"))
        for d in doc["landmarks"]:
            reg.add(_lm_from(d))
        for name, r in (doc.get("routines") or {}).items():
            if not r.get("steps"):
                raise ValueError(f"routine {name}: no steps")
            reg.routines[name] = {"aliases": r.get("aliases", []), "steps": r["steps"],
                                  "description": r.get("description", "")}
        reg.default_routine = doc.get("default_routine")
        if reg.default_routine and reg.default_routine not in reg.routines:
            raise ValueError(f"default_routine {reg.default_routine!r} is not a routine")
        reg.taught_path = taught_path
        if taught_path and os.path.exists(taught_path):
            with open(taught_path) as f:
                for d in (yaml.safe_load(f) or {}).get("landmarks", []):
                    lm = _lm_from(d); lm.taught = True
                    if lm.name in reg._lm and not reg._lm[lm.name].taught:
                        continue                  # curated config always wins
                    reg.add(lm)
        return reg

    def teach(self, name, x, y, yaw, aliases=(), type_="room", gauge=None, position=None):
        """Remember a place: by default the robot's CURRENT pose ("this is the kitchen"),
        stored as an approach pose. Found gauges (find_gauges) pass the dial's own map
        `position` and its `gauge` spec, with (x, y, yaw) = the viewing spot. Persisted
        to the taught overlay. Refuses to overwrite a curated (config-file) landmark."""
        cur = self._lm.get(name)
        if cur is not None and not cur.taught and cur.type != "home":
            raise ValueError(f"{name!r} is a surveyed landmark; teach a different name")
        self.add(Landmark(name, type_, tuple(position) if position else (x, y), (x, y, yaw),
                          aliases=list(aliases), gauge=dict(gauge) if gauge else None,
                          taught=True))
        self._save_taught()
        return self._lm[name]

    def set_gauge_scale(self, text, lo, hi, units):
        """'DIAL-004 is 0 to 100 psi': give a FOUND gauge its real scale, so readings are
        logged in units instead of a fraction of the dial. Curated gauges keep their
        surveyed scale (not changeable by voice)."""
        lm = self.resolve(text)
        if lm is None or lm.type != "gauge":
            raise ValueError(f"no gauge called {text!r}")
        if not lm.taught:
            raise ValueError(f"{lm.name} is a surveyed gauge; its scale is in the landmark file")
        if not float(lo) < float(hi):
            raise ValueError(f"scale {lo}..{hi} is empty")
        lm.gauge = {"units": str(units), "min": float(lo), "max": float(hi)}
        self._save_taught()
        return lm

    def _save_taught(self):
        if self.taught_path:
            taught = [lm for lm in self._lm.values() if lm.taught]
            doc = {"landmarks": [dict({"name": lm.name, "type": lm.type, "aliases": lm.aliases,
                                       "position": [round(float(v), 3) for v in lm.position],
                                       "approach": [round(float(v), 4) for v in lm.approach]},
                                      **({"gauge": lm.gauge} if lm.gauge else {}))
                                 for lm in taught]}
            os.makedirs(os.path.dirname(os.path.abspath(self.taught_path)), exist_ok=True)
            tmp = self.taught_path + ".tmp"
            with open(tmp, "w") as f:
                yaml.safe_dump(doc, f)
            os.replace(tmp, self.taught_path)      # atomic: a crash never corrupts it

    def resolve_routine(self, text):
        t = (text or "").strip().lower()
        if t in ("", "mission", "the mission", "default", "routine") and self.default_routine:
            return self.default_routine
        for name, r in self.routines.items():
            if t == name.lower() or t.replace(" ", "_") == name.lower() or \
                    t in (a.lower() for a in r["aliases"]):
                return name
        return None

    def add(self, lm):
        if lm.type not in TYPES:
            raise ValueError(f"landmark {lm.name}: unknown type {lm.type!r}")
        if len(lm.approach) != 3:
            raise ValueError(f"landmark {lm.name}: approach must be [x, y, yaw]")
        if lm.type == "gauge":
            g = lm.gauge or {}
            if not {"units", "min", "max"} <= set(g):
                raise ValueError(f"gauge {lm.name}: needs gauge.units/min/max")
        self._lm[lm.name] = lm

    def set_home(self, x, y, yaw):
        self._lm["home"] = Landmark("home", "home", (x, y), (x, y, yaw),
                                    aliases=["start", "where you started"])

    def get(self, name):
        return self._lm.get(name)

    def resolve(self, text):
        """Exact name, then case-insensitive alias match. None if unknown."""
        if text in self._lm:
            return self._lm[text]
        t = text.strip().lower()
        for lm in self._lm.values():
            if t == lm.name.lower() or t in (a.lower() for a in lm.aliases):
                return lm
        return None

    def of_type(self, t):
        return [lm for lm in self._lm.values() if lm.type == t]

    def names(self):
        return list(self._lm)

    def nearest_order(self, names, start_xy):
        """Greedy nearest-neighbour visiting order (cheap TSP) over approach poses."""
        left = [self._lm[n] for n in names]
        out, cur = [], start_xy
        while left:
            nxt = min(left, key=lambda lm: math.hypot(lm.approach[0] - cur[0],
                                                      lm.approach[1] - cur[1]))
            out.append(nxt.name); left.remove(nxt); cur = nxt.approach[:2]
        return out

    def describe(self):
        out = "\n".join(lm.summary() for lm in self._lm.values())
        if self.routines:
            lines = []
            for n, r in self.routines.items():
                tag = ' (DEFAULT: what "the mission" means)' if n == self.default_routine else ""
                lines.append(f"{n}{tag} aka {', '.join(r['aliases'])}: {r['description']}")
            out += "\n\nSaved routines (use run_routine):\n" + "\n".join(lines)
        return out
