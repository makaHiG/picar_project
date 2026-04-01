class Mode:
    IDLE="idle"
    MANUAL ="manual"
    DIRECTIONAL_MOVE="directional move"

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