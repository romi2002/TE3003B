import numpy as np
from sklearn.neighbors import NearestNeighbors
from line_profiler import profile

class LikelihoodField():
    @profile
    def __init__(self, occupancy_grid, search_cell, max_range=10):
        # For all cells in map, pre-compute distance to nearest occupied cell
        map_data = occupancy_grid.get_map() > 0.5

        # Limit search size to only include coordinates within max_range + 1 of pose.
        sx, sy = int(max(0, search_cell[0] - max_range)), int(max(0, search_cell[1] - max_range))
        ex, ey = int(min(map_data.shape[0], search_cell[0] + max_range)), int(
            min(map_data.shape[1], search_cell[1] + max_range))
        self.start_coords = (sx, sy)
        self.end_coords = (ex, ey)
        X = np.argwhere((np.isreal(map_data)[sx:ex, sy:ey]))

        occupied_cells = np.argwhere((map_data == 0)[sx:ex, sy:ey])

        self.nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto', n_jobs=-1).fit(occupied_cells)

        # Coordinate of all grid cells
        dist, idx = self.nbrs.kneighbors(X)

        print(search_cell, (sx, sy), (ex, ey), (ex - sx, ey - sy))
        self.closest_obstacle = dist.reshape(ex - sx, ey - sy)

    def closest_obstacle_to_pose(self, x, y):
        # Since we do not calculate the whole map, fetch from matrix with offset
        search_x = int(x - self.start_coords[0])
        search_y = int(y - self.start_coords[1])
        assert self.start_coords[0] < x <= self.end_coords[0], "X is outside of search range"
        assert self.start_coords[1] < y <= self.end_coords[1], "Y is outside of search range"
        return self.closest_obstacle[search_x, search_y]
