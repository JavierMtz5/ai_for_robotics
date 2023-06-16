from math import *
import random
from typing import List
from particle_filter.robot import Robot
from search.astar import search_astar
from pid_control.constrained_smoothing import smooth

steering_noise = 0.1
distance_noise = 0.03
measurement_noise = 0.3


class Planning:

    def __init__(self, grid: List[List[int]], init: List[int],
                 goal: List[int], cost=1.):
        """
        Planning class is used to calculate the optimal path from
        initial state to goal, using A* algorithm
        Arguments:
            grid: 2D List containing 1s and 0s, where 0 is a navigable state and 1 is a wall
            init: Initial state as [x, y]
            goal: Goal state as [x, y]
            cost: Cost of taking a step
        """
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.heuristic = [[]]
        self.make_heuristic()
        self.optimal_path = list()
        self.smoothed_path = list()

    def make_heuristic(self) -> None:
        """
        Creates a 2D grid where each cell contains the value of the heuristic function
        for that state. In this case the heuristic function is the Manhattan distance
        from each state to the goal
        """
        # Initialize the heuristic with the same size as the grid
        self.heuristic = [[0 for _ in range(len(self.grid[0]))]
                          for _ in range(len(self.grid))]
        # Set value of each state as Manhattan distance to the goal
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + abs(j - self.goal[1])

    def astar(self) -> None:
        """
        Applies A* algorithm to find the optimum path from the initial position to the goal state.
        """
        # Check if heuristic grid is filled
        if not self.heuristic:
            raise ValueError("Heuristic must be defined to run A*")

        possible_actions = [[-1, 0],  # go up
                            [0, -1],  # go left
                            [1, 0],  # go down
                            [0, 1]]  # do right
        actions_symbols = ['^', '<', 'v', '>']
        expand, policy, optimal_path = search_astar(self.grid, self.init, self.goal, self.cost,
                                                    self.heuristic, possible_actions, actions_symbols)
        self.optimal_path = optimal_path

    def smooth(self, weight_data: float = 0.1, weight_smooth: float = 0.1,
               tolerance: float = 0.000001) -> None:
        """
        Smooth the optimal path previously calculated with the A* algorithm
        Arguments:
            weight_data: weigh for gradient descent
            weight_smooth: weigh for gradient descent
            tolerance: When the difference between the last smoothed value and the new one is less
                than tolerance, the smoothing process ends
        """
        # Check that the optimal path has been calculated
        if not self.optimal_path:
            raise ValueError("Run A* first before smoothing path")

        self.smoothed_path = smooth(path=self.optimal_path,
                                    fix=[0 for _ in range(len(self.optimal_path))],
                                    weight_data=weight_data,
                                    weight_smooth=weight_smooth,
                                    tolerance=tolerance)


class ParticleFilter:

    def __init__(self, x: float, y: float, theta: float, n=100):
        """
        Create Particle Filter with the parameters given as parameters
        Arguments:
            x: initial x coordinate for the particles (initial x position of the actual robot)
            y: initial y coordinate for the particles (initial y position of the actual robot)
            theta: initial orientation for the particles (initial orientation of the actual robot)
            n:number of particles to generate
        """
        self.n = n

        self.particles = []
        for i in range(self.n):
            particle = Robot()
            particle.set(x, y, theta)
            particle.set_noise(0., 0., 0., 0., steering_noise, distance_noise, measurement_noise, 0.)
            self.particles.append(particle)

    def get_particles_position(self) -> List[float]:
        """
        Returns the mean x, y and orientation of all the particles
        """
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.n):
            x += self.particles[i].x
            y += self.particles[i].y
            orientation += ((self.particles[i].orientation
                             - self.particles[0].orientation + pi) % (2.0 * pi)) + self.particles[0].orientation - pi
        mean_x, mean_y, mean_orientation = x / self.n, y / self.n, orientation / self.n
        return [mean_x, mean_y, mean_orientation]

    def move(self, steer: float, speed: float) -> None:
        """
        Applies motion to every particle and updates the particles list
        """
        new_particles = list()

        for i in range(self.n):
            # Apply motion on the particle
            self.particles[i].circular_move(steer, speed)

    def sense_and_resample(self, Z: List[float]) -> None:

        weights = list()
        # Calculate the weight of each of the particles
        for i in range(self.n):
            weights.append(self.particles[i].measurement_prob_robot_position(Z))

        # Resample using Resampling Wheel algorithm
        resampled_particles = list()
        index = int(random.random() * self.n)  # Pick random index
        beta = 0.0
        max_weight = max(weights)

        for i in range(self.n):
            beta += random.random() * 2.0 * max_weight
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.n
            resampled_particles.append(self.particles[index])
        self.particles = resampled_particles


def run(grid, goal, spath, params, steering_noise: float, distance_noise: float,
        measurement_noise: float, speed=0.1, timeout=1000):
    """
    Run localization on a robot
    """
    robot = Robot()
    robot.set(0., 0., 0.)
    robot.set_noise(0., 0., 0., 0., steering_noise, distance_noise, measurement_noise, 0.)
    p_filter = ParticleFilter(robot.x, robot.y, robot.orientation)

    index = 0  # Index into the path
    cte = 0.  # Track crosstrack error
    err = 0.0  # Track error during localization
    n = 0  # Track number of steps

    while not robot.check_goal(goal) and n < timeout:

        # Calculate steering angle with PID controller
        diff_cte = -cte
        estimate = p_filter.get_particles_position()
        x, y, orientation = estimate
        u, cte = robot.segmented_cte(x, y, spath, index)
        if u > 1.:
            index += 1
        diff_cte += cte

        steer = - params[0] * cte - params[1] * diff_cte

        # Apply motion on robot and on particles
        robot.circular_move(steer, speed)
        p_filter.move(steer, speed)

        # Sense the robot position, and resample particles with the sensing
        Z = robot.sense_robot_position()
        p_filter.sense_and_resample(Z)

        if not robot.check_collision(grid):
            print('##### Collision ####')

        err += (cte ** 2)
        n += 1

    return [robot.check_goal(goal), robot.num_collisions, n]


def main() -> None:
    # Define the parameters to be used
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 1],
            [0, 1, 0, 1, 0, 0]]
    init = [0, 0]
    goal = [len(grid) - 1, len(grid[0]) - 1]
    steering_n = 0.1
    distance_n = 0.03
    measurement_n = 0.3

    weight_data = 0.1
    weight_smooth = 0.2
    p_gain = 2.0
    d_gain = 6.0

    path = Planning(grid, init, goal)
    path.astar()
    path.smooth(weight_data, weight_smooth)
    print(run(grid, goal, path.smoothed_path, [p_gain, d_gain],
              steering_n, distance_n, measurement_n))


if __name__ == '__main__':
    main()
