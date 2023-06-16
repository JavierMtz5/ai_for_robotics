from math import *
from typing import List, Union
import random
import numpy as np
from particle_filter.robot import Robot


def make_data(n: int,
              num_landmarks: int,
              world_size: float,
              measurement_range: float,
              motion_noise: float,
              measurement_noise: float,
              distance: float) -> List[List[Union[List[List[float]], List[float]]]]:
    """
    Creates random world data for later performing Graph SLAM
    Arguments:
        n: number of timesteps
        num_landmarks: number of landmarks
        world_size: size fo the world (world_size x world_size)
        measurement_range: max distance that the robot can sense
        motion_noise: noise when applying motion
        measurement_noise: noise when sensing
        distance: distance to move the robot
    """
    # Complete will be set to True when the robot has seen every landmark on the map
    complete = False
    while not complete:

        data = list()

        # Create robot and world landmarks, and set initial position to middle of the world
        robot = Robot(world_size=world_size,
                      measurement_range=measurement_range,
                      motion_noise=motion_noise,
                      measurement_noise=measurement_noise)
        robot.set(world_size / 2., world_size / 2., 0.)
        robot.make_landmarks(num_landmarks)
        seen = [False for _ in range(num_landmarks)]

        # Get an initial orientation and motion randomly
        # The x and y relative distances are calculated previously to store them in data list
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance

        # Iterate as many times as timesteps
        for k in range(n - 1):

            # Sense
            Z = robot.sense_x_y()

            # Check how many of the landmarks are seen by the robot
            for i in range(len(Z)):
                seen[Z[i][0]] = True

            # Set robot on the state required to perform the motion
            robot.set(robot.x, robot.y, orientation)
            # Apply motion to the robot
            while not robot.move(0, distance):
                # if we'd be leaving the robot world, pick instead a new direction
                orientation = random.random() * 2.0 * pi
                dx = cos(orientation) * distance
                dy = sin(orientation) * distance

                # Set robot on the state required to retry the motion
                robot.set(robot.x, robot.y, orientation)

            # Add the movement and the sensing to data list
            data.append([Z, [dx, dy]])

        # If all landmarks are seen at least once, the data generation ends
        complete = (sum(seen) == num_landmarks)

    # Print data about final robot and landmark position
    print('\nGenerated data:')
    for landmark in robot.landmarks:
        print(f'Actual Landmark position: {landmark}')
    print(f'Actual state of the robot: [x={robot.x} y={robot.y}]')
    print('\n')

    return data


def slam(data: List[List[Union[List[List[float]], List[float]]]],
         n: int,
         num_landmarks: int,
         motion_noise: float,
         measurement_noise: float) -> np.ndarray:
    """
    Performs the Linear Graph SLAM algorithm
    Arguments:
        data: List containing the data about each timestep taken by the actual robot
            Each element is a List which contains info about the sensing and the motion.
            The first element (sense data) is another List containing the index of the landmark
            being sensed, and the x and y distance from robot to landmark. The second element is
            another List containing the x and y distance travelled by the robot from the previous state
        n: number of timesteps
        num_landmarks: number of landmarks in the world
        motion_noise: noise when applying motion
        measurement_noise: noise when sensing
    """
    # Initialize Omega and Xi matrices for Graph SLAM
    omega = np.zeros((2 * n + 2 * num_landmarks, 2 * n + 2 * num_landmarks))
    xi = np.zeros((2 * n + 2 * num_landmarks, 1))

    # Set initial pose constraint on both Omega and Xi
    omega[0][0], omega[1][1] = 1., 1.
    xi[0][0], xi[1][0] = 50., 50.

    # Set measurement and motion constraints for each timestep taken by the robot
    for index, data_batch in enumerate(data):

        measurement = data_batch[0]
        dx, dy = data_batch[1]

        # Define x and y indices to update (index of previous state and current one)
        index_x_prev, index_x_post = index * 2, index * 2 + 2
        index_y_prev, index_y_post = index * 2 + 1, index * 2 + 3

        # Set motion constraint on Omega for x-axis
        omega[index_x_prev][index_x_prev] += 1. / motion_noise
        omega[index_x_post][index_x_prev] += -1 * (1. / motion_noise)
        omega[index_x_prev][index_x_post] += -1 * (1. / motion_noise)
        omega[index_x_post][index_x_post] += 1. / motion_noise

        # Set motion constraint on Xi for x-axis
        xi[index_x_prev][0] += (1. / motion_noise) * -1 * dx
        xi[index_x_post][0] += (1. / motion_noise) * dx

        # Set motion constraint on Omega for y-axis
        omega[index_y_prev][index_y_prev] += 1. / motion_noise
        omega[index_y_post][index_y_prev] += -1 * (1. / motion_noise)
        omega[index_y_prev][index_y_post] += -1 * (1. / motion_noise)
        omega[index_y_post][index_y_post] += 1. / motion_noise

        # Set motion constraint on Xi for y-axis
        xi[index_y_prev][0] += (1. / motion_noise) * -1 * dy
        xi[index_y_post][0] += (1. / motion_noise) * dy

        for measure in measurement:
            # Define Lx and Ly indices to update (index of each landmark)
            index_L_x = 2 * (n + measure[0])
            index_L_y = 2 * (n + measure[0]) + 1

            # Set measurement constraints on Omega for x-axis
            omega[index_x_prev][index_x_prev] += 1. / measurement_noise
            omega[index_x_prev][index_L_x] += -1 * (1. / measurement_noise)
            omega[index_L_x][index_x_prev] += -1 * (1. / measurement_noise)
            omega[index_L_x][index_L_x] += 1. / measurement_noise

            # Set measurement constraints on Xi for x-axis
            xi[index_x_prev][0] += -1 * (1. / measurement_noise) * measure[1]
            xi[index_L_x][0] += (1. / measurement_noise) * measure[1]

            # Set measurement constraints on Omega for y-axis
            omega[index_y_prev][index_y_prev] += 1. / measurement_noise
            omega[index_y_prev][index_L_y] += -1 * (1. / measurement_noise)
            omega[index_L_y][index_y_prev] += -1 * (1. / measurement_noise)
            omega[index_L_y][index_L_y] += 1. / measurement_noise

            # Set measurement constraints on Xi for y-axis
            xi[index_y_prev][0] += -1 * (1. / measurement_noise) * measure[2]
            xi[index_L_y][0] += (1. / measurement_noise) * measure[2]

    mu = np.linalg.inv(omega) @ xi

    return mu


def main() -> None:

    num_landmarks = 5
    n = 20
    world_size = 100.0
    measurement_range = 50.0
    motion_noise = 2.0
    measurement_noise = 2.0
    distance = 20.0

    data = make_data(n, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)
    result = slam(data, n, num_landmarks, motion_noise, measurement_noise)

    # Convert the result to visible data
    for i in range(0, len(result), 2):
        index_x, index_y = i, i + 1
        if i < len(result) - num_landmarks*2:
            print(f'[x={result[index_x]} y={result[index_y]}]')
        else:
            print(f'Landmark location: [x={result[index_x]} y={result[index_y]}]')


if __name__ == '__main__':
    main()
