import math

import matplotlib.pyplot as plt
import numpy as np

from slam.likelihood_field import LikelihoodField
from slam.occupancy_grid import OccupancyGrid
from slam.particle import Particle
from slam.scan import Scan
from line_profiler import profile
from concurrent.futures import ThreadPoolExecutor


# Probabilistic robotics, Table 13.4 P 478
class FastSLAM:
    def __init__(self):
        self.dt = 0.1
        self.num_particles = 15
        self.particles = [Particle(None) for _ in range(self.num_particles)]
        self.best_map = None
        self.best_pose = None

    def sample_normal(self, b):
        return np.random.normal(loc=0.0, scale=max(1e-6, np.sqrt(b)))

    def plot_particles(self, particles, map_data):
        fig, ax = plt.subplots()
        for particle in particles:
            ax.scatter(particle.pose[0], particle.pose[1], c='r', marker='o')
        ax.imshow(map_data)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Particle Poses')

        plt.show()

    # Given state x (x,y,theta) and control u (v, omega), sample a pose.
    def sample_motion_model(self, u, x):
        x, y, theta = x
        v, omega = u

        # Parameters of motion noise.
        a1, a2, a3, a4, a5, a6 = [0.005] * 6

        v_hat = v + self.sample_normal(a1 * (v ** 2) + a2 * (omega ** 2))
        omega_hat = omega + self.sample_normal(a3 * (v ** 2) + a4 * (omega ** 2))
        gamma_hat = self.sample_normal(a5 * (v ** 2) + a6 * (omega ** 2))

        x_prime = x - (v_hat / omega_hat) * np.sin(theta) + (v_hat / omega_hat) * np.sin(theta + omega_hat * self.dt)
        y_prime = y + (v_hat / omega_hat) * np.cos(theta) - (v_hat / omega_hat) * np.cos(theta + omega_hat * self.dt)
        theta_prime = theta + omega_hat * self.dt + gamma_hat * self.dt
        return np.array([x_prime, y_prime, theta_prime])

    @staticmethod
    def prob_centered_gaussian(mean, variance):
        return np.exp(-(np.power(mean, 2) / variance / 2.0) / np.sqrt(2.0 * np.pi * variance))
        # c = 1.0 / (std_dev * np.sqrt(2 * math.pi))
        # prob = c * math.exp((-math.pow(dist, 2)) / (2 * math.pow(std_dev, 2)))
        # return prob

    # Computes range model using liklihood field
    # Given scan, pose and map
    # Table 6.3 P.143
    def measurement_model_map(self, scan: Scan, pose, grid: OccupancyGrid, like_field: LikelihoodField):
        x, y, theta = pose
        q = 1
        for i, range in enumerate(scan.ranges):
            if range >= scan.range_max:
                continue

            angle = (i * scan.angle_increment) + scan.angle_min
            assert (angle < scan.angle_max)
            angle += theta

            # Get the reported position of the obstacle
            x_z = x + range * np.cos(angle + theta)
            y_z = y + range * np.sin(angle + theta)

            x_c, y_c = grid.coordinate_to_cell(x_z, y_z)

            # Now get the distance to the closest obstacle
            try:
                distance = like_field.closest_obstacle_to_pose(x_c, y_c)
            except AssertionError:
                distance = 10

            # Update probability, this is using the simplified model, without taking into account z_random and z_max
            # TODO, Maybe implement this?
            sigma_hit = 20
            # print(f"dist: {distance} prob: {FastSLAM.prob_centered_gaussian(distance, np.power(sigma_hit, 2))}")
            q = q * FastSLAM.prob_centered_gaussian(distance, np.power(sigma_hit, 2))
        return q

    def process_particle(self, particle, u, pose, scan):
        # Initialize pose.
        if particle.pose is None:
            particle.pose = pose

        # Sample_motion_model
        particle.pose = self.sample_motion_model(u, pose)
        print(f"pose: {pose} sampled: {particle.pose} delta: {particle.pose - pose}")
        if self.best_map is None:
            particle.map = OccupancyGrid()
            particle.map.update(particle.pose, scan)
        else:
            particle.map = self.best_map

        # Measurement_model_map
        search_cell = particle.map.coordinate_to_cell(particle.pose[0], particle.pose[1])
        particle.like_field = LikelihoodField(particle.map, search_cell,
                                              max_range=scan.range_max / particle.map.map_resolution + 10)
        #particle.like_field.debug_plot()
        particle.weight = self.measurement_model_map(scan, particle.pose, particle.map, particle.like_field)
        # particle.like_field.debug_plot()
        # updated_occupancy_grid
        particle.map.update(particle.pose, scan)

    @profile
    def update(self, u, pose, scan):
        # Sample motion model and create particles

        # Fill with new particles
        while len(self.particles) < self.num_particles:
            self.particles.append(Particle(pose, self.best_map))

        with ThreadPoolExecutor() as executor:
            for particle in self.particles:
                executor.submit(self.process_particle, particle, u, pose, scan)

        # for particle in self.particles:
        #     self.process_particle(particle, u, pose, scan)
        # Normalize, and sample particles based off weight
        weights = np.array([p.weight for p in self.particles])
        print(weights)
        if not np.all(weights == 0):
            # print(weights)
            weights = weights / np.sum(weights)

            # low variance sampler for particles p.86
            resampled_indexes = set()
            r = np.random.rand() * (1 / len(self.particles))
            c = weights[0]
            i = 0
            for m in range(len(self.particles)):
                u = r + (m - 1) * (1 / len(self.particles))
                while u > c:
                    i += 1
                    c += weights[i]
                resampled_indexes.add(i)

            # Prune such that only particles in resampled_indexes are kept
            self.particles = [p for i, p in enumerate(self.particles) if i in resampled_indexes]
        else:
            print("particle weights are zero")

        # Compute unweighted (?) average of state
        # out_pose = np.zeros(3)
        # out_map = np.zeros(self.particles[0].map.log_prob_map.shape)
        # for particle in self.particles:
        #     out_pose += particle.pose
        #     out_map += particle.map.log_prob_map
        # out_pose /= len(self.particles)
        # out_map /= len(self.particles)

        # Choose best particle?
        best_particle_idx = np.argmax([p.weight for p in self.particles])
        print(f"Best particle: {best_particle_idx} weight: {self.particles[best_particle_idx].weight}")
        self.best_map = self.particles[best_particle_idx].map
        self.best_pose = self.particles[best_particle_idx].pose
        return self.particles[best_particle_idx].pose, self.particles[best_particle_idx].map.get_map()
        return out_pose, 1 - 1 / (1 + np.exp(np.clip(out_map, -10, 10)))
