class SetMode:
    class Request:
        def __init__(self): self.mode=0; self.requester=""
    class Response:
        def __init__(self): self.accepted=False; self.reason=""

class SetControlAuthority:
    class Request:
        def __init__(self): self.tier=0; self.requester=""; self.lease_ms=0
    class Response:
        def __init__(self): self.granted=False; self.reason=""
