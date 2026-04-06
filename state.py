class Mode:
    IDLE="idle"
    MANUAL ="manual"
    DIRECTIONAL_MOVE="directional move"
    ORIENTING = "orienting"

class RobotState:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.rotation = 0
        self.mode = Mode.IDLE
        self.targetAngle = 0
        self.corridorAngle=0
        self.right_distance = 100
        self.left_distance = 100
        self.front_distance = 100
        self.scan = ScanState()
class ScanState:
    def __init__(self):
        self.readings=[]
        self.active = False
        self.startRotation = 0
        
