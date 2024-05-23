import numpy as np
from slam.scan import Scan
import matplotlib.pyplot as plt
from tqdm import tqdm
from slam.occupancy_grid import OccupancyGrid
from slam.dataset import load_dataset

scans, poses, vels = load_dataset('orebro.gfs.log')
# scans = scans[::4]
# poses = poses[::4]
# vels = vels[::4]
from slam.fast_slam import FastSLAM
slam = FastSLAM()
out_poses = []
maps = []
print(f"Total poses: {len(poses)}")

grid = OccupancyGrid()
estimated_angular_vel = np.diff(poses[:,2], axis=0)

delta_pose = np.diff(poses[:,:2], axis=0)
estimated_vel = np.vstack(
    (np.zeros(2),
    np.vstack((np.hypot(delta_pose[:,0], delta_pose[:,1]), np.diff(poses[:,2], axis=0))).T)
)
print(estimated_vel)
for i in tqdm(range(len(poses))):
    print(estimated_vel[i])
    pose, out_map = slam.update(estimated_vel[i], poses[i], scans[i])
    grid.update(poses[i], scans[i])
    out_poses.append(pose)
    print(f"in pose: {poses[i]} vel: {vels[i]} est_vel: {estimated_vel[i]} out pose: {pose}")
    maps.append(out_map)

fig, (ax1, ax2) = plt.subplots(figsize=(32,32), nrows=2)
for particle in slam.particles:
    x, y = particle.map.coordinate_to_cell(
        particle.pose[0],
        particle.pose[1]
    )
    print(x,y)
    ax1.scatter(y, x, c='r', marker='o', s=1)

ax1.imshow(maps[-1])
particle_poses = np.array([slam.particles[0].map.coordinate_to_cell(*p[:2]) for p in out_poses])
#ax1.plot(*zip(*np.vstack((particle_poses[:,1], particle_poses[:,0]))))
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Particle Poses Final')

ax2.imshow(grid.get_map(), cmap='gray')

plt.show()
