#!/usr/bin/env python3
"""Deep-merge a YAML overlay onto a ROS params file (null in the overlay DELETES a key).

    python3 tools/nav/apply_overlay.py base.yaml overlay.yaml out.yaml
Used by bringup for M20_SITE=office (src/m20_navigation/config/nav2_office_overlay.yaml).
Fails loudly if an overlay path doesn't exist in the base, except for explicitly NEW
leaf keys (e.g. footprint). A typo in a node name must not silently do nothing.
"""
import sys

import yaml


def merge(base, over, path=""):
    for k, v in over.items():
        p = f"{path}/{k}"
        if v is None:
            base.pop(k, None)
        elif isinstance(v, dict):
            if k not in base:
                raise KeyError(f"overlay section {p} not in base params (typo?)")
            merge(base[k], v, p)
        else:
            base[k] = v
    return base


class NoAlias(yaml.SafeDumper):
    def ignore_aliases(self, data):   # rcl's parser rejects anchors/aliases
        return True


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    base = yaml.safe_load(open(sys.argv[1]))
    over = yaml.safe_load(open(sys.argv[2]))
    yaml.dump(merge(base, over), open(sys.argv[3], "w"), Dumper=NoAlias, sort_keys=False)
    print(f"merged {sys.argv[2]} -> {sys.argv[3]}")


if __name__ == "__main__":
    main()
