from math import *
import random
from typing import *

from robot import Robot


def evaluate_particles(bot: Robot, particles: List[Robot], world_size: float) -> float:
    """Evaluates the particles by measuring the mean distance from the particles to the actual robot"""
    tot = 0.0
    for i in range(len(particles)):
        # Compute distance between particles and actual robot, which acts as error
        dx = (particles[i].x - bot.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
        dy = (particles[i].y - bot.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
        err = sqrt(dx * dx + dy * dy)
        tot += err

    # Return the mean error over all particles
    return tot / float(len(particles))


def main() -> None:

    world_size_data = 100.0
    landmarks_data = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]

    # Initialize actual Robot
    robot = Robot(world_size_data, landmarks_data)
    robot = robot.move(0.1, 5.0)
    n = 1000
    t = 10

    particles = list()

    # Generate n particles with forward_noise 0.05, turn_noise 0.05 and sense noise 5, and store them in list particles
    for _ in range(n):
        r = Robot(world_size_data, landmarks_data)
        r.set_noise(0.05, 0.05, 5.0, 0., 0., 0.)
        particles.append(r)

    # Iterate through the Robot's motions and sensings to compute the position of the particles
    for timestep in range(t):

        # Perform motion on actual Robot and sense
        robot = robot.move(0.1, 5.0)
        sensing = robot.sense()

        # Perform motion on particles and update particles list
        particles_after_motion = list()
        for i in range(n):
            particles_after_motion.append(particles[i].move(0.1, 5.0))
        particles = particles_after_motion

        # Weight each particle according to how probable it is to be in the actual
        # robot's location, based on the actual robot's sensing
        weighted_particles = list()
        for i in range(n):
            particle_weight = particles[i].measurement_prob(sensing)
            weighted_particles.append(particle_weight)

        # Resample particles with Resampling Wheel algorithm
        resampled_particles = list()
        index = int(random.random() * n)
        beta = 0.0
        max_weight = max(weighted_particles)
        for i in range(n):
            beta += random.random() * 2.0 * max_weight
            while beta > weighted_particles[index]:
                beta -= weighted_particles[index]
                index = (index + 1) % n
            resampled_particles.append(particles[index])
        particles = resampled_particles

        print(f'Particles mean error for timestep {timestep} : {evaluate_particles(robot, particles, world_size_data)}')


if __name__ == '__main__':
    main()
