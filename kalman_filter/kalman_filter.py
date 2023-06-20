import numpy as np
from typing import *


def kalman_filter(measurements: List[int], initial_state: np.ndarray, P: np.ndarray,
                  u: np.ndarray, F: np.ndarray, H: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Multi-dimensional Kalman Filter
    Arguments:
        measurements: list containing the measure of the robot position on each timestep
        (its only able to measure position)
        initial_state: initial state [location, velocity]
        P: initial uncertainty covariance
        u: external motion vector
        F: state transition matrix
        H: measurement function
        R: measurement uncertainty
    """
    I = np.identity(2)
    estimate = initial_state
    for measurement in measurements:

        # Measurement update
        y = np.subtract(np.array([[measurement]]), H @ estimate)
        S = np.add(H @ P @ H.T, R)
        K = P @ H.T @ np.linalg.inv(S)
        estimate = np.add(estimate, K @ y)
        P = np.subtract(I, K @ H) @ P

        # Prediction
        estimate = np.add(F @ estimate, u)
        P = F @ P @ F.T

    return estimate, P


def main() -> None:
    measurement_seq_1 = [1, 2, 3]
    measurement_seq_2 = [1, 2, 3, 4, 5, 6, 7]
    measurement_seq_3 = [1, 3, 5, 7]

    x = np.array([[0.], [0.]])
    P = np.array([[1000., 0.], [0., 1000.]])
    u = np.array([[0.], [0.]])
    F = np.array([[1., 1.], [0, 1.]])
    H = np.array([[1., 0.]])
    R = np.array([[1.]])

    print('\nThe first array represents the expected position and velocity of the robot, respectively. '
          'The second one represents the uncertainty covariance matrix P', end='\n\n')

    print('Solution for measurement sequence 1: ', kalman_filter(measurement_seq_1, x, P, u, F, H, R), end='\n\n')
    print('Solution for measurement sequence 2: ', kalman_filter(measurement_seq_2, x, P, u, F, H, R), end='\n\n')
    print('Solution for measurement sequence 3: ', kalman_filter(measurement_seq_3, x, P, u, F, H, R), end='\n\n')


if __name__ == '__main__':
    main()
