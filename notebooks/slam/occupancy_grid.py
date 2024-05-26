import numpy as np
import matplotlib.pyplot as plt

from slam.scan import Scan
from line_profiler import profile

class OccupancyGrid():
    def __init__(self):
        # Defined in meters
        self.map_size = (100, 100)
        self.map_resolution = 0.1
        self.map_origin = (-50, -50)
        self.log_prob_map = np.zeros([
            int(self.map_size[0] / self.map_resolution),
            int(self.map_size[1] / self.map_resolution)],
            dtype=np.float32
        )
        self.test_map = np.zeros([
            int(self.map_size[0] / self.map_resolution),
            int(self.map_size[1] / self.map_resolution)],
            dtype=np.float32
        )

    def coordinate_to_cell(self, x, y):
        return (
            (x - self.map_origin[0]) / self.map_resolution,
            (y - self.map_origin[1]) / self.map_resolution
        )

    def is_valid_cell_coordinate(self, coordinate):
        x_c, y_c = coordinate
        return (
                (0 < x_c < self.log_prob_map.shape[0]) and
                (0 < y_c < self.log_prob_map.shape[1])
        )

    # Returns grid cells between points using Bresenham's line algorithm
    # https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    @staticmethod
    @profile
    def bresenham_line(x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))
        return np.array(points)

    # Converts a laser scan around pose and returns a 2D array where each point represents where a laser beam has hit an obstacle.
    def process_scan(self, pose, scan: Scan):
        x, y, theta = pose

        scan_angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        return np.array((
            x + scan.ranges * np.cos(scan_angles + theta),
            y + scan.ranges * np.sin(scan_angles + theta)
        )).T

    @profile
    def update_cells(self, pose, scan):
        x, y, theta = pose
        # Get the occupied cells from scans with an obstacle
        obstacle_cells = self.process_scan(pose, scan)

        # Free cells are now given by pose and line drawn from pose to obstacle_cells
        free_cells = np.array([[x, y], [x, y]])
        for i, (x_obstacle, y_obstacle) in enumerate(obstacle_cells):
            x_c, y_c = self.coordinate_to_cell(x_obstacle, y_obstacle)
            if scan.ranges[i] >= scan.range_max or not self.is_valid_cell_coordinate((x_c, y_c)):
                continue
            free_cells = np.vstack((free_cells,
                                    OccupancyGrid.bresenham_line(
                                        *np.round(self.coordinate_to_cell(x, y)),
                                        np.round(x_c), np.round(y_c))))

        return [self.coordinate_to_cell(x, y) for (x, y) in obstacle_cells], free_cells

    @staticmethod
    # Returns the log prob of an occupied cell or free cell
    def prob(occupied):
        probability = 0.75
        A, B = probability, 1 - probability
        if occupied:
            return np.log(B / A)
        else:
            return np.log(A / B)

    def get_map(self):
        return 1 - 1 / (1 + np.exp(np.clip(self.log_prob_map, -10, 10)))

    def debug_plot(self, pose, obstacle_cells, free_cells, show_cells=True):
        fig, ax = plt.subplots(figsize=(32,32))
        ax.imshow(self.get_map())
        ax.set_title("occupancy grid")
        if show_cells:
            ax.scatter(*zip(*free_cells), s=1, c='b')
        ax.scatter(*self.coordinate_to_cell(pose[0], pose[1]), s=10, c='g', marker="^")
        if show_cells:
            ax.scatter(*zip(*obstacle_cells), s=1, c='r')
        plt.show()

    @profile
    def update(self, pose, scan: Scan):
        obstacle_cells, free_cells = self.update_cells(pose, scan)
        for (x, y) in filter(self.is_valid_cell_coordinate, obstacle_cells):
            self.log_prob_map[int(x), int(y)] += self.prob(True)
            self.test_map[int(x), int(y)] = 1
        for (x, y) in filter(self.is_valid_cell_coordinate, free_cells):
            self.log_prob_map[int(x), int(y)] += self.prob(False)
            self.test_map[int(x), int(y)] = 0
        # self.debug_plot(pose, obstacle_cells, free_cells, show_cells=False)
