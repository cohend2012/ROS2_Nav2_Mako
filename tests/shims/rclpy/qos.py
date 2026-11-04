class ReliabilityPolicy: RELIABLE = 1; BEST_EFFORT = 2
class DurabilityPolicy: TRANSIENT_LOCAL = 1; VOLATILE = 2
class HistoryPolicy: KEEP_LAST = 1; KEEP_ALL = 2
class QoSProfile:
    def __init__(self, **kw): pass
qos_profile_sensor_data = QoSProfile()
