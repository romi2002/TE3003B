import numpy as np
from slam.scan import Scan

# Load from dataset for testing
def load_dataset(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    laser_data = []
    odometry_data = []
    vel_data = []

    for line in lines:
        if line.startswith('FLASER'):
            line = line.strip().split()
            num_laser_values = int(line[1])  # Extracting number of laser values
            values = line[2:]  # Extracting laser and odometry values
            max_range = 10
            scan_data = np.clip([float(value) for value in values[:num_laser_values]], 0, max_range + 1)
            scan = Scan(angle_min=-np.pi / 2, angle_max=np.pi / 2, range_max=max_range,
                        angle_increment=(np.pi) / float(num_laser_values), ranges=scan_data)
            laser_data.append(scan)
            # laser_data.append(scan_data)
            odometry_data.append([float(value) for value in values[num_laser_values:num_laser_values + 3]])
            vel_data.append([float(v) for v in values[num_laser_values + 3:num_laser_values + 5]])

    return np.array(laser_data), np.array(odometry_data), np.array(vel_data)