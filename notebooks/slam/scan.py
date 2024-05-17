class Scan():
    def __init__(self, angle_min, angle_max, angle_increment, ranges, range_max):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        assert((angle_max - angle_min) / angle_increment == len(ranges))
        self.ranges = ranges
        self.range_max = range_max