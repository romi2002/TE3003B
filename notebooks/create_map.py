import numpy as np
from slam.scan import Scan
import matplotlib.pyplot as plt
from tqdm.contrib import tzip
from slam.dataset import load_dataset
from slam.occupancy_grid import OccupancyGrid

scans, poses, vels = load_dataset('orebro.gfs.log')

grid = OccupancyGrid()
start = 0
total_count = 1
for pose, scan in tzip(poses[start:start + total_count], scans[start:start + total_count]):
    grid.update(pose, scan)
plt.imshow(grid.get_map() > 0.5, cmap='gray')
plt.show()
# ax = plt.gca()
# ax.set_xlim([0, 300])
# ax.set_ylim([0, 100])
# plt.savefig('out/%03d.png' % i)