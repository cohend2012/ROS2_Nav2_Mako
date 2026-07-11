import os, sys
HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "shims"))
for pkg in ["m20_commander", "m20_locomotion_bridge", "m20_behaviors", "m20_missions"]:
    sys.path.insert(0, os.path.join(ROOT, "src", pkg))

import pytest
import rclpy

@pytest.fixture(autouse=True)
def fresh_bus():
    rclpy.reset()
    yield
