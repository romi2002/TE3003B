class Particle:
    def __init__(self, pose, in_map=None):
        self.pose = pose
        self.weight = 0
        self.map = in_map
        self.like_field = None
