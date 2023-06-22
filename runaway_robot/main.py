from math import *
from typing import List, Any
from particle_filter.robot import Robot
import numpy as np


def estimate_next_pos(measurement: List[float], OTHER: Any = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    if OTHER is None:
        OTHER = list()
        # States are stored in OTHER as (x, y, orient, turning_angle, velocity)
        OTHER.append([measurement[0], measurement[1], 0., 0., 0.])
        xy_estimate = measurement[0], measurement[1]

    else:
        if len(OTHER) == 1:
            # Calculate the orientation from the last state to the previous one, and update last state
            motion_orientation = atan2(measurement[1] - OTHER[-1][1], measurement[0] - OTHER[-1][0])
            OTHER[-1][2] = motion_orientation

            # Calculate the speed of the robot, and update last state
            motion_velocity = (measurement[0] - OTHER[-1][0]) / cos(motion_orientation)
            OTHER[-1][4] = motion_velocity

            # Update OTHER with the new state
            OTHER.append([measurement[0], measurement[1], 0., 0., 0.])

            # Compute the expected new state
            new_x = measurement[0] + motion_velocity * cos(motion_orientation)
            new_y = measurement[1] + motion_velocity * sin(motion_orientation)
            xy_estimate = new_x, new_y

        else:
            motion_orientation = atan2(measurement[1] - OTHER[-1][1], measurement[0] - OTHER[-1][0])
            OTHER[-1][2] = motion_orientation

            # Calculate the speed of the robot, and update last state
            motion_velocity = (measurement[0] - OTHER[-1][0]) / cos(motion_orientation)
            OTHER[-1][4] = motion_velocity

            # Calculate the turning angle of the robot
            motion_turning_angle = motion_orientation - OTHER[-2][2]

            # Update OTHER with the new state
            OTHER.append([measurement[0], measurement[1], motion_orientation, motion_turning_angle, motion_velocity])

            # Compute the expected new state
            new_x = measurement[0] + motion_velocity * cos(motion_orientation + motion_turning_angle)
            new_y = measurement[1] + motion_velocity * sin(motion_orientation + motion_turning_angle)
            xy_estimate = new_x, new_y

    return xy_estimate, OTHER


def estimate_next_pos_noisy(measurement: List[float], OTHER: Any = None):
    """
    Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements
    Arguments:
        measurement: List containing x and y coordinates of the measurement
        OTHER: List used to store the estimate, P matrix and measurements of previous motions
    """
    # dt = 1.
    # # Initialize Identity matrix and external motion vector
    # I = np.identity(5)
    # H = np.array([[1., 0., 0., 0., 0.],
    #               [0., 1., 0., 0., 0.]])
    # R = np.array([[measurement_noise, 0.],
    #               [0., measurement_noise]])
    #
    # if not OTHER:
    #
    #     # Set initial estimate
    #     estimate = np.zeros((5, 1))
    #     # Set initial P uncertainty covariance matrix
    #     P = np.zeros((5, 5))
    #     np.fill_diagonal(P, 1000.)
    #     # Initialize OTHER List
    #     OTHER = [None, None, []]
    #
    # else:
    #     # Get previous P and estimate from OTHER List
    #     P, estimate = OTHER[0], OTHER[1]
    #
    # measurement_arr = np.array([measurement])
    # # Measurement update
    # y = np.subtract(measurement_arr.T, H @ estimate)
    # S = np.add(H @ P @ H.T, R)
    # K = P @ H.T @ np.linalg.inv(S)
    # estimate = np.add(estimate, K @ y)
    # P = np.subtract(I, K @ H) @ P
    #
    # # Get current estimates to construct the transition matrix F
    # x0, y0, orient0, beta0, vel0 = estimate[0][0], estimate[1][0], estimate[2][0], estimate[3][0], estimate[4][0]
    # F = np.array([[1., 0., -vel0 * sin(orient0 + beta0), -vel0 * sin(orient0 + beta0), cos(orient0 + beta0)],
    #               [0., 1., vel0 * cos(orient0 + beta0), vel0 * cos(orient0 + beta0), sin(orient0 + beta0)],
    #               [0., 0., 1., dt, 0.],
    #               [0., 0., 0., 1., 0.],
    #               [0., 0., 0., 0., 1.]])
    # estimate = np.array([[x0 + vel0 * cos(orient0 + beta0)],
    #                      [y0 + vel0 * sin(orient0 + beta0)],
    #                      [(orient0+beta0) % 2*pi],
    #                      [beta0],
    #                      [vel0]])
    #
    # # Prediction
    # # estimate = np.add(F @ estimate, u)
    # P = F @ P @ F.T
    #
    # # Store the computed P matrix and estimate, and add the new measurement
    # OTHER[0], OTHER[1] = P, estimate
    #
    # # Get estimate of the position as x and y tuple
    # xy_estimate = estimate[0][0], estimate[1][0]
    # return xy_estimate, OTHER

    x1 = measurement[0]
    y1 = measurement[1]

    if not OTHER:
        OTHER = [[], [], []]
        # inital guesses:
        x0 = 0.
        y0 = 0.
        dist0 = 0.
        theta0 = 0.
        dtheta0 = 0.
        # initial uncertainty:
        P = np.array([[1000., 0., 0., 0., 0.],
                      [0., 1000., 0., 0., 0.],
                      [0., 0., 1000., 0., 0.],
                      [0., 0., 0., 1000., 0.],
                      [0., 0., 0., 0., 1000.]])
    else:
        # pull previous measurement, state variables (x), and uncertainty (P) from OTHER
        x0 = OTHER[0][0][0]
        y0 = OTHER[0][1][0]
        dist0 = OTHER[0][2][0]
        theta0 = OTHER[0][3][0] % (2 * pi)
        dtheta0 = OTHER[0][4][0]
        P = OTHER[1]

    # time step
    dt = 1.

    # state matrix (polar location and angular velocity)
    x = np.array([[x0], [y0], [dist0], [theta0], [dtheta0]])

    # measurement function:
    # for the EKF this should be the Jacobian of H, but in this case it turns out to be the same (?)
    H = np.array([[1., 0., 0., 0., 0.],
                  [0., 1., 0., 0., 0.]])
    # measurement uncertainty:
    R = np.array([[measurement_noise, 0.],
                  [0., measurement_noise]])
    # 5d identity matrix
    I = np.identity(5)

    # measurement update
    Z = np.array([[x1, y1]])
    y = Z.T - H @ x
    S = np.add(H @ P @ H.T, R)
    K = P @ H.T @ np.linalg.inv(S)
    x = np.add(x, (K @ y))
    P = np.subtract(I, K @ H) @ P

    # pull out current estimates based on measurement
    x0 = x[0][0]
    y0 = x[1][0]
    dist0 = x[2][0]
    theta0 = x[3][0]
    dtheta0 = x[4][0]

    # next state function:
    # this is now the Jacobian of the transition matrix (F) from the regular Kalman Filter
    A = np.array([[1., 0., cos(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0)],
                  [0., 1., sin(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0)],
                  [0., 0., 1., 0., 0.],
                  [0., 0., 0., 1., dt],
                  [0., 0., 0., 0., 1.]])

    # calculate new estimate
    # it's NOT simply the matrix multiplication of transition matrix and estimated state vector
    # for the EKF just use the state transition formulas the transition matrix was built from
    x = np.array([[x0 + dist0 * cos(theta0 + dtheta0)],
                  [y0 + dist0 * sin(theta0 + dtheta0)],
                  [dist0],
                  [theta0 + dtheta0],
                  [dtheta0]])

    # prediction
    # x = (F * x) + u
    P = A * P * A.transpose()

    OTHER[0] = x
    OTHER[1] = P

    # print "x:"
    # x.show()
    # print "P:"
    # P.show()

    xy_estimate = (x[0][0], x[1][0])

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance = 1.5
    distance_tolerance = 0.01 * distance
    ctr = 0

    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense_robot_position()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot = target_bot.circular_move(steering=2 * pi / 10, distance=distance)
        true_position = (target_bot.x, target_bot.y)
        print('Position guess: ', position_guess)
        print('True position: ', true_position, end='\n\n')
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 10:
            print("Sorry, it took you too many steps to localize the target.")

    return localized


def demo_grading_noisy(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance = 1.5
    distance_tolerance = 0.01 * distance
    ctr = 0

    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense_robot_position()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot = target_bot.circular_move(steering=2 * pi / 34., distance=distance)
        true_position = (target_bot.x, target_bot.y)
        print('Position guess: ', position_guess)
        print('True position: ', true_position, end='\n\n')
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print("Sorry, it took you too many steps to localize the target.")
    return localized


def main():
    test_target = Robot()
    test_target.set(2.1, 4.3, 0.)
    test_target.set_noise(0., 0., 0., 0., 0., 0., 0.)

    demo_grading(estimate_next_pos, test_target)


def main_noisy():
    measurement_noise = 0.05 * 1.5
    test_target = Robot()
    test_target.set(2.1, 4.3, 0.5)
    test_target.set_noise(0., 0., 0., 0., 0., measurement_noise, 0.)

    demo_grading_noisy(estimate_next_pos_noisy, test_target)


if __name__ == '__main__':
    measurement_noise = 0.05 * 1.5
    main_noisy()
