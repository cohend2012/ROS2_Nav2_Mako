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

`home` is NOT in the file: the executor records it at mission start (set_home).
Runtime perception adds entries with add(); the registry is the agent's ONLY
vocabulary for motion targets — a name that isn't here cannot be driven to.
"""
import math
from dataclasses import dataclass, field

import yaml

TYPES = ("gauge", "tank", "valve", "wellhead", "skid", "rack", "area", "home")


@dataclass
class Landmark:
    name: str
    type: str
    position: tuple
    approach: tuple
    aliases: list = field(default_factory=list)
    gauge: dict = None

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

    @classmethod
    def load(cls, path):
        with open(path) as f:
            doc = yaml.safe_load(f)
        if not doc or "landmarks" not in doc:
            raise ValueError(f"{path}: no 'landmarks' list")
        reg = cls(doc.get("frame", "map"))
        for d in doc["landmarks"]:
            reg.add(Landmark(name=d["name"], type=d["type"], position=tuple(d["position"]),
                             approach=tuple(d["approach"]), aliases=d.get("aliases", []),
                             gauge=d.get("gauge")))
        return reg

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
        return "\n".join(lm.summary() for lm in self._lm.values())
