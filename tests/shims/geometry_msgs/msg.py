class _V3:
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
class Twist:
    def __init__(self): self.linear = _V3(); self.angular = _V3()
class PoseStamped:
    def __init__(self): self.header = None
