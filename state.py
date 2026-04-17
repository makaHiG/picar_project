class Mode:
    IDLE="idle"
    MANUAL ="manual"
    DIRECTIONAL_MOVE="directional move"
    ORIENTING = "orienting"
    SPINNING = "spinning"

class RobotState:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.direction = 0
        self.rotation = 0
        self.mode = Mode.IDLE
        self.targetAngle = 0
        self.corridorAngle=0
        self.center_errors =[]
        self.align_errors = []
        self.readings = []
        self.right_distance = 100
        self.left_distance = 100
        self.front_distance = 100
        self.scan = ScanState()
        self.spinn = SpinnState()
        self.lastPhotoSpot = (0,0)
class ScanState:
    def __init__(self):
        self.readings=[]
        self.active = False
        self.startRotation = 0

class SpinnState:
    def __init__(self):
        self.targetRotation = 0
        self.active = False
        self.stepCount = 0
        self.maxSteps = 12
        self.startRotation = 0

        
