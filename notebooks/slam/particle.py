class Particle:
    def __init__(self, pose):
        self.pose = pose
        self.weight = 0
        self.map = None
        self.like_field = None
