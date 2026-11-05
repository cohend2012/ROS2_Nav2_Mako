#!/usr/bin/env python3
"""Deep-merge a YAML overlay onto a ROS params file (null in the overlay DELETES a key).

    python3 tools/nav/apply_overlay.py base.yaml overlay.yaml out.yaml
    python3 tools/nav/apply_overlay.py base.yaml over1.yaml [over2.yaml ...] out.yaml
Used by bringup for M20_SITE=office (+ M20_NAV_VARIANT=<name> for the classical
variants in src/m20_navigation/config/office_variants/). Overlays apply in order.
Fails loudly if an overlay names a NODE section that is not in the base (a typo in a
node name must not silently do nothing); new nested parameter blocks inside an
existing node are allowed (a new controller plugin brings its own sub-params).
"{share:<pkg>}" in any string is replaced by that package's installed share dir.
"""
import sys

import yaml


def share_dir(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001 — no ament index: search the prefix path
        import os
        for pre in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            d = os.path.join(pre, "share", pkg)
            if pre and os.path.isdir(d):
                return d
        raise KeyError(f"{{share:{pkg}}}: package not found (source ROS first)")


def expand(v):
    import re
    if isinstance(v, str):
        return re.sub(r"\{share:([A-Za-z0-9_]+)\}", lambda m: share_dir(m.group(1)), v)
    if isinstance(v, list):
        return [expand(x) for x in v]
    return v


def merge(base, over, path=""):
    for k, v in over.items():
        p = f"{path}/{k}"
        if v is None:
            base.pop(k, None)
        elif isinstance(v, dict):
            if k not in base:
                if path == "":
                    raise KeyError(f"overlay node section {p} not in base params (typo?)")
                base[k] = {}
            if not isinstance(base[k], dict):
                base[k] = {}
            merge(base[k], v, p)
        else:
            base[k] = expand(v)
    return base


class NoAlias(yaml.SafeDumper):
    def ignore_aliases(self, data):   # rcl's parser rejects anchors/aliases
        return True


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    base = yaml.safe_load(open(sys.argv[1]))
    for o in sys.argv[2:-1]:
        merge(base, yaml.safe_load(open(o)))
    yaml.dump(base, open(sys.argv[-1], "w"), Dumper=NoAlias, sort_keys=False)
    print(f"merged {' + '.join(sys.argv[2:-1])} -> {sys.argv[-1]}")


if __name__ == "__main__":
    main()
