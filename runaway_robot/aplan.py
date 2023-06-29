import numpy as np
from utils.robot import Robot
from utils.utils import (
    distance_between,
    get_heading,
    angle_trunc,
    test_pose_estimation,
    test_and_visualize_noisy_pose_estimation,
    test_robot_capture
)

from typing import List, Any, Tuple, Optional, Union
from math import cos, sin, pi, atan2


def next_move_slow_hunter(hunter_position: List[float], hunter_heading: float,
                          target_measurement: List[float], measurement_noise: float,
                          max_distance: float, previous_iteration_data: Any = None) -> Tuple[float, float, List[list]]:
    """
    Returns the next move to be performed by the hunter robot in order to reach the target robot
    Arguments:
        hunter_position: List of 2 elements: x and y coordinates of the hunter robot
        hunter_heading: orientation of the hunter robot in radians
        target_measurement: List of 2 elements: x and y coordinates of the target robot
        measurement_noise: measurement noise
        max_distance: max distance to be travelled by the hunter robot
        previous_iteration_data: List containing data about the previous iteration's calculations.
            The format of this data is [kalman_P, kalman_estimate]
    """
    # Get the estimated next state of the target robot using a Kalman Filter
    xy_estimate, P_and_estimate = estimate_next_pos_noisy(measurement=target_measurement,
                                                          measurement_noise=measurement_noise,
                                                          previous_iteration_data=previous_iteration_data)

    # Store the new P matrix and estimate vector in previous_iteration_data for next iterations
    P, estimate = P_and_estimate
    if previous_iteration_data is None:
        previous_iteration_data = [None, None]
    previous_iteration_data[0] = P
    previous_iteration_data[1] = estimate

    target = xy_estimate
    theta = (estimate[2][0] - estimate[3][0]) % (2 * pi)
    vel = estimate[4][0]
    beta = estimate[3][0]

    # The hunter robot will calculate which is the next target state (one or many steps ahead)
    # that it can reach according to its speed. If the target robot will get faster to a state,
    # then the hunter robot checks the next target position. If the hunter robot will get faster to a
    # future target state than the target robot, then it aims for that position
    i = 1
    while distance_between(hunter_position, target) > max_distance * i:
        i += 1
        target = (target[0] + vel * cos(theta + beta), target[1] + vel * sin(theta + beta))
        theta = angle_trunc(theta + beta)
        if i > 1000:
            break

    # Compute the distance to be travelled by the hunter as the distance between hunter and target
    distance = distance_between(hunter_position, target)
    if distance > max_distance:
        distance = max_distance

    # Compute the steering angle of the hunter so its orientation is the same one as the target robot's
    diff_heading = get_heading(hunter_position, target)
    turning = angle_trunc(diff_heading - hunter_heading)
    return turning, distance, previous_iteration_data


def next_move_fast_hunter(hunter_position: List[float], hunter_heading: float,
                          target_measurement: List[float], measurement_noise: float,
                          max_distance: float,
                          previous_iteration_data: List[Union[List[np.ndarray], List[float], float]] = None
                          ) -> Tuple[float, float, List[Union[List[np.ndarray], List[float], float]]]:
    """
    Returns the next move to be performed by the hunter robot in order to reach the target robot
    Arguments:
        hunter_position: List of 2 elements: x and y coordinates of the hunter robot
        hunter_heading: orientation of the hunter robot. Not used in this function,
            but present for refactor purpose
        target_measurement: List of 2 elements: x and y coordinates of the target robot
        measurement_noise: measurement noise
        max_distance: max distance to be travelled by the hunter robot
        previous_iteration_data: List containing data about the previous iteration's calculations.
            The format of this data is
            [[kalman_P, kalman_estimate], target_previous_position, previous_cte]
    """
    # Set values for the PID Controller
    tau_p, tau_d = 2.0, 6.0

    # If there is no previous data, initialize the Kalman data (P and estimate) to None, the previous
    # position of the target to the current hunter position and the previous crosstrack error to 0
    if not previous_iteration_data:
        kalman_data = None
        target_prev_pos = hunter_position
        prev_cte = 0.
        previous_iteration_data = [None, None, None]

    else:
        kalman_data = previous_iteration_data[0]
        target_prev_pos = previous_iteration_data[1]
        prev_cte = previous_iteration_data[2]

    # First, the next position of the target robot is predicted according to the Kalman filter
    target_next_pos, kalman_data = estimate_next_pos_noisy(target_measurement, measurement_noise, kalman_data)

    # The line between the previous target pos and the new target pos will be the
    # line to follow using a PID controller
    path = [target_prev_pos, target_next_pos]
    diff_cte = -prev_cte
    u, cte = Robot().segmented_cte(hunter_position[0], hunter_position[1],  # Calculate cte
                                   path, 0)
    diff_cte += cte

    # Calculate steering angle using the PID Controller
    turning = -tau_p * cte - tau_d * diff_cte

    # Compute the distance to be travelled by the hunter as the distance between hunter and target
    distance = distance_between(hunter_position, target_next_pos)
    if distance > max_distance:
        distance = max_distance

    # Store data in previous_iteration_data for next iteration
    previous_iteration_data[0] = kalman_data
    previous_iteration_data[1] = target_next_pos
    previous_iteration_data[2] = cte

    return turning, distance, previous_iteration_data


def estimate_next_pos(measurement: List[float],
                      previous_iteration_data: Optional[List[List[float]]] = None
                      ) -> Tuple[Tuple[float, float], List[List[float]]]:
    """
    Estimate the next (x, y) position of the Robot based on exact measurements received from the robot
    - Calculates the distance in each step as the distance between the new state and the previous
    one (to calculate this at least 2 steps are required).
    - Calculates the turning angle in each step as the difference between the robot orientation in the current
    state and the previous one.
    - Once the distance per step and the steering angle per step are known, the new x and y positions of the
    robot can be calculated as:
            x' = x + dist * cos(orientation + turning angle)
            y' = y + dist * sin(orientation + turning angle)
    Arguments:
        measurement: List of 2 floats, which correspond to the x and y coordinates of the robot.
            This measurement is not noisy, so it represent the real state of the robot.
        previous_iteration_data: List containing data about the previous iteration's measurement calculations.
            The format of this data is
            [[x0, y0, orient0, turning_angle0, distance0], [x1, y1, orient1, turning_angle1, distance1], ...]
    """

    # If it is the first iteration no previous data exists, and only the current x and y measurements can be stored
    if previous_iteration_data is None:

        previous_iteration_data = list()
        previous_iteration_data.append([measurement[0], measurement[1], 0., 0., 0.])
        xy_estimate = measurement[0], measurement[1]

    else:

        # If it is the second iteration, then the previous x and y measurements are available, and so
        # the orientation and velocity of the robot can be calculated
        if len(previous_iteration_data) == 1:

            # Calculate the orientation from the last state to the previous one, and update last state
            motion_orientation = atan2(measurement[1] - previous_iteration_data[-1][1],
                                       measurement[0] - previous_iteration_data[-1][0])
            previous_iteration_data[-1][2] = motion_orientation

            # Calculate the speed of the robot, and update last state
            motion_velocity = (measurement[0] - previous_iteration_data[-1][0]) / cos(motion_orientation)
            previous_iteration_data[-1][4] = motion_velocity

            # Update previous_iteration_data with the new state
            previous_iteration_data.append([measurement[0], measurement[1], 0., 0., 0.])

            # Compute the expected new state
            new_x = measurement[0] + motion_velocity * cos(motion_orientation)
            new_y = measurement[1] + motion_velocity * sin(motion_orientation)
            xy_estimate = new_x, new_y

        # If more than two iterations are present in previous_iteration_data, then the turning
        # angle of the robot can be calculated by comparing the previous orientation and the current one.
        # With all this information, the next robot pose can be predicted
        else:

            # Calculate the orientation from the last state to the previous one, and update last state
            motion_orientation = atan2(measurement[1] - previous_iteration_data[-1][1],
                                       measurement[0] - previous_iteration_data[-1][0])
            previous_iteration_data[-1][2] = motion_orientation

            # Calculate the speed of the robot, and update last state
            motion_velocity = (measurement[0] - previous_iteration_data[-1][0]) / cos(motion_orientation)
            previous_iteration_data[-1][4] = motion_velocity

            # Calculate the turning angle of the robot
            motion_turning_angle = motion_orientation - previous_iteration_data[-2][2]

            # Update previous_iteration_data with the new state
            previous_iteration_data.append(
                [measurement[0], measurement[1], motion_orientation, motion_turning_angle, motion_velocity])

            # Compute the expected new state
            new_x = measurement[0] + motion_velocity * cos(motion_orientation + motion_turning_angle)
            new_y = measurement[1] + motion_velocity * sin(motion_orientation + motion_turning_angle)
            xy_estimate = new_x, new_y

    return xy_estimate, previous_iteration_data


def estimate_next_pos_noisy(measurement: List[float],
                            measurement_noise: float,
                            previous_iteration_data: Optional[List[np.ndarray]] = None
                            ) -> Tuple[Tuple[float, float], List[np.ndarray]]:
    """
    Estimate the next (x, y) position of the Robot based on noisy (x, y) measurements by applying Kalman filter
    Arguments:
        measurement: List of 2 floats, which correspond to the x and y coordinates of the robot.
            This measurement is noisy, so it may not represent the real state of the robot.
        measurement_noise: measurement noise
        previous_iteration_data: List containing data about the previous iteration's measurement calculations.
            The format of this data is
            [P, estimate], being P a matrix of size 5x5 and estimate a vector of size 5
    """
    dt = 1.
    # Initialize Identity matrix I, Measurement Function Matrix H and Measurement Uncertainty Matrix R
    I = np.identity(5)
    H = np.array([[1., 0., 0., 0., 0.],
                  [0., 1., 0., 0., 0.]])
    R = np.array([[measurement_noise, 0.],
                  [0., measurement_noise]])

    # If there is no previous data, both the initial P and estimate are initialized
    if not previous_iteration_data:

        # Set initial estimate
        estimate = np.zeros((5, 1))
        # Set initial P uncertainty covariance matrix
        P = np.zeros((5, 5))
        np.fill_diagonal(P, 1000.)
        # Initialize previous_iteration_data List
        previous_iteration_data = [None, None]

    # If previous data exists, P and estimate are taken as the previous iteration's P and estimate
    else:

        P, estimate = previous_iteration_data[0], previous_iteration_data[1]

    measurement_arr = np.array([measurement])

    # Measurement update
    y = np.subtract(measurement_arr.T, H @ estimate)
    S = np.add(H @ P @ H.T, R)
    K = P @ H.T @ np.linalg.inv(S)
    estimate = np.add(estimate, K @ y)
    P = np.subtract(I, K @ H) @ P

    # Get current estimates to construct the transition matrix F
    x0, y0, orient0, beta0, vel0 = estimate[0][0], estimate[1][0], estimate[2][0], estimate[3][0], estimate[4][0]
    F = np.array([[1., 0., -vel0 * sin(orient0 + beta0), -vel0 * sin(orient0 + beta0), cos(orient0 + beta0)],
                  [0., 1., vel0 * cos(orient0 + beta0), vel0 * cos(orient0 + beta0), sin(orient0 + beta0)],
                  [0., 0., 1., dt, 0.],
                  [0., 0., 0., 1., 0.],
                  [0., 0., 0., 0., 1.]])
    estimate = np.array([[x0 + vel0 * cos(orient0 + beta0)],
                         [y0 + vel0 * sin(orient0 + beta0)],
                         [(orient0 + beta0) % (2 * pi)],
                         [beta0],
                         [vel0]])

    # Prediction
    P = F @ P @ F.T

    # Store the computed P matrix and estimate, and add the new measurement
    previous_iteration_data[0], previous_iteration_data[1] = P, estimate

    # Get estimate of the position as x and y tuple
    xy_estimate = estimate[0][0], estimate[1][0]

    return xy_estimate, previous_iteration_data


def estimate_next_target_pos_without_noise() -> None:
    target = Robot()
    target.set(2.1, 4.3, 0.)
    test_pose_estimation(estimate_next_pos, target)


def estimate_next_target_pos_with_noise() -> None:
    measurement_noise = 0.075
    target = Robot()
    target.set(2.1, 4.3, 0.5)
    target.set_noise(0., 0., 0., 0., 0., measurement_noise, 0.)
    test_and_visualize_noisy_pose_estimation(estimate_next_pos_noisy, target)


def capture_target_robot_with_high_velocity() -> None:
    measurement_noise = 0.075
    target = Robot()
    target.set(0., 10., 0.)
    target.set_noise(0., 0., 0., 0., 0., measurement_noise, 0.)
    hunter = Robot()
    hunter.set(-10., -10., 0.)
    test_robot_capture(hunter, target, next_move_fast_hunter, 1.97)


def capture_target_robot_with_low_velocity() -> None:
    measurement_noise = 0.075
    target = Robot()
    target.set(0., 10., 0.)
    target.set_noise(0., 0., 0., 0., 0., measurement_noise, 0.)
    hunter = Robot()
    hunter.set(-10., -10., 0.)
    test_robot_capture(hunter, target, next_move_slow_hunter, 0.97)


if __name__ == '__main__':
    # Estimate the Target Robot pose without noise
    print('Estimate for the Target Robot pose without noise:')
    estimate_next_target_pos_without_noise()
    print('\n')

    # Estimate the Target Robot pose with measurement noise
    print('Estimate for the Target Robot pose with noise:')
    estimate_next_target_pos_with_noise()
    print('\n')

    # Capture the Target Robot when Hunter Robot's max velocity is almost twice the Target's max velocity
    print('Capture the Target Robot with high velocity:')
    capture_target_robot_with_high_velocity()
    print('\n')

    # Capture the Target Robot when Hunter Robot's max velocity is lower than Target's max velocity
    print('Capture the Target Robot with low velocity:')
    capture_target_robot_with_low_velocity()
    print('\n')
