
from slam.particle import Particle
import numpy as np
import matplotlib.pyplot as plt
import math
from slam.scan import Scan
from slam.likelihood_field import LikelihoodField
from slam.occupancy_grid import OccupancyGrid

# Probabilistic robotics, Table 13.4 P 478
class FastSLAM():
    def __init__(self):
        self.dt = 0.01
        self.num_particles = 1
        self.particles = [Particle(None) for _ in range(self.num_particles)]

    def sample_normal(self, b):
        return np.random.normal(loc=0.0, scale=max(1e-6, np.sqrt(b)))

    def plot_particles(self, particles):
        fig, ax = plt.subplots()
        for particle in particles:
            ax.scatter(particle.pose[0], particle.pose[1], c='r', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Particle Poses')

        plt.show()

    # Given state x (x,y,theta) and control u (v, omega), sample a pose.
    def sample_motion_model(self, u, x):
        x, y, theta = x
        v, omega = u

        # Parameters of motion noise.
        a1, a2, a3, a4, a5, a6 = [0.01] * 6

        v_hat = v + self.sample_normal(a1 * (v ** 2) + a2 * (omega ** 2))
        omega_hat = omega + self.sample_normal(a3 * (v ** 2) + a4 * (omega ** 2))
        gamma_hat = self.sample_normal(a5 * (v ** 2) + a6 * (omega ** 2))

        x_prime = x - (v_hat / omega_hat) * np.sin(theta) + (v_hat / omega_hat) * np.sin(theta + omega_hat * self.dt)
        y_prime = y + (v_hat / omega_hat) * np.cos(theta) - (v_hat / omega_hat) * np.cos(theta + omega_hat * self.dt)
        theta_prime = theta + omega_hat * self.dt + gamma_hat * self.dt
        return np.array([x_prime, y_prime, theta_prime])

    @staticmethod
    def prob_centered_gaussian(dist, std_dev):
        c = 1.0 / (std_dev * np.sqrt(2 * math.pi))
        prob = c * math.exp((-math.pow(dist ,2) ) /(2 * math.pow(std_dev, 2)))
        return prob

    # Computes range model using liklihood field
    # Given scan, pose and map
    # Table 6.3 P.143
    def measurement_model_map(self, scan: Scan, pose, like_field: LikelihoodField):
        x, y, theta = pose
        q = 1
        for i, range in enumerate(scan.ranges):
            if range >= scan.range_max:
                continue

            angle = (i * scan.angle_increment) + scan.angle_min
            assert(angle < scan.angle_max)
            angle += theta

            # Get the reported position of the obstacle
            x_z = x + range * np.cos(angle)
            y_z = y + range * np.sin(angle)

            # Now get the distance to the closest obstacle
            distance = like_field.closest_obstacle_to_pose(x_z, y_z)

            # Update probability, this is using the simplified model, without taking into account z_random and z_max
            # TODO, Maybe implement this?
            q = q * FastSLAM.prob_centered_gaussian(distance, 0.01)
        return q

    def update(self, u, pose, scan):
        # Sample motion model and create particles
        for particle in self.particles:
            # Initialize pose.
            if particle.pose is None:
                particle.pose = pose

            # Sample_motion_model
            particle.pose = self.sample_motion_model(u, particle.pose)
            if particle.map is None:
                particle.map = OccupancyGrid()

            # Measurement_model_map
            search_cell = particle.map.coordinate_to_cell(pose[0], pose[1])
            particle.like_field = LikelihoodField(particle.map, search_cell, max_range=scan.range_max)
            particle.weight = self.measurement_model_map(scan, particle.pose, particle.like_field)

            # updated_occupancy_grid
            particle.map.update(particle.pose, scan)