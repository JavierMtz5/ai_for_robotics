from typing import List, Union, Tuple
import numpy as np
from slam.graph_slam_2d import make_data, slam
from utils.utils import plot_graph_slam_estimates


def online_slam(data: List[List[Union[List[List[float]], List[float]]]],
                num_landmarks: int,
                motion_noise: float,
                measurement_noise: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Performs the Linear Graph SLAM algorithm on a scalable way, which maintains the same size for
    both Omega and Xi matrices
    Arguments:
         data: List containing the data about each timestep taken by the actual robot
            Each element is a List which contains info about the sensing and the motion.
            The first element (sense data) is another List containing the index of the landmark
            being sensed, and the x and y distance from robot to landmark. The second element is
            another List containing the x and y distance travelled by the robot from the previous state
         num_landmarks: number of landmarks in the world
         motion_noise: noise when applying motion
         measurement_noise: noise when sensing
    """
    # Initialize Omega and Xi matrices for Scalable Graph SLAM
    omega = np.zeros((2 * (num_landmarks + 1), 2 * (num_landmarks + 1)))
    xi = np.zeros((2 * (num_landmarks + 1), 1))

    # Set initial pose constraint on both Omega and Xi
    omega[0][0] += 1
    omega[1][1] += 1
    xi[0][0] += 50.
    xi[1][0] += 50.

    # Iterate through every data measured (motion + measurement)
    for data_batch in data:
        measurement = data_batch[0]
        dx, dy = data_batch[1]

        # Set measurement contraints
        for measure in measurement:
            # Define Lx and Ly indices to update (index of each landmark)
            index_L_x = 2 + 2 * measure[0]
            index_L_y = 3 + 2 * measure[0]

            # Set measurement constraints on Omega for x-axis
            omega[0][0] += 1. / measurement_noise
            omega[index_L_x][index_L_x] += 1. / measurement_noise
            omega[0][index_L_x] += -1. / measurement_noise
            omega[index_L_x][0] += -1. / measurement_noise

            # Set measurement constraints on Xi for x-axis
            xi[0][0] += -1. * measure[1] * (1. / measurement_noise)
            xi[index_L_x][0] += measure[1] / measurement_noise

            # Set measurement constraints on Omega for y-axis
            omega[1][1] += 1. / measurement_noise
            omega[index_L_y][index_L_y] += 1. / measurement_noise
            omega[1][index_L_y] += -1. / measurement_noise
            omega[index_L_y][1] += -1. / measurement_noise

            # Set measurement constraints on Xi for y-axis
            xi[1][0] += -1. * measure[2] * (1. / measurement_noise)
            xi[index_L_y][0] += measure[2] / measurement_noise

        # Expand Omega and Xi to hold the constraints for the new motion
        omega = np.insert(omega, [2, 2], 0, axis=0)
        omega = np.insert(omega, [2, 2], 0, axis=1)
        xi = np.insert(xi, [2, 2], 0, axis=0)

        # Set motion constraint on Omega for x-axis
        omega[0][0] += 1. / motion_noise
        omega[2][0] += -1. / motion_noise
        omega[0][2] += -1. / motion_noise
        omega[2][2] += 1. / motion_noise

        # Set motion constraint on Xi for x-axis
        xi[0][0] += -1. * dx * (1. / motion_noise)
        xi[2][0] += dx / motion_noise

        # Set motion constraint on Omega for y-axis
        omega[1][1] += 1. / motion_noise
        omega[1][3] += -1. / motion_noise
        omega[3][3] += 1. / motion_noise
        omega[3][1] += -1. / motion_noise

        # Set motion constraint on Xi for y-axis
        xi[1][0] += -1. * dy * (1. / motion_noise)
        xi[3][0] += dy / motion_noise

        # Resize the matrix to the original shape to allow rescaling
        omega_prima = omega[2:, 2:]
        xi_prima = xi[2:]
        A = omega[:2, 2:]
        B = omega[:2, :2]
        C = xi[:2]

        omega = np.subtract(omega_prima, A.T @ np.linalg.inv(B) @ A)
        xi = np.subtract(xi_prima, A.T @ np.linalg.inv(B) @ C)

    Omega = omega
    mu = np.linalg.inv(Omega) @ xi

    return mu, Omega


def main() -> None:
    num_landmarks = 5
    n = 20
    world_size = 100.0
    measurement_range = 50.0
    motion_noise = 2.0
    measurement_noise = 2.0
    distance = 20.0

    data, plot_data = make_data(n, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)
    slam_mu = slam(data, n, num_landmarks, motion_noise, measurement_noise)

    # Convert the result to visible data
    print('Results of GRAPH SLAM')
    for i in range(0, len(slam_mu), 2):
        index_x, index_y = i, i + 1
        if i < len(slam_mu) - num_landmarks * 2:
            if i == len(slam_mu) - 2 * num_landmarks - 2:
                print(f'Robot location:    [x={slam_mu[index_x]} y={slam_mu[index_y]}]')
            else:
                print(f'[x={slam_mu[index_x]} y={slam_mu[index_y]}]')
        else:
            print(f'Landmark location: [x={slam_mu[index_x]} y={slam_mu[index_y]}]')
    print('\n\n')

    # Plot the position of the real robot and landmarks, and the estimate position of the real robot ,
    # and landmarks. Also plot the path followed by the robot during the SLAM process
    plot_graph_slam_estimates(slam_mu, plot_data, num_landmarks)

    scalable_slam_mu, scalable_slam_omega = online_slam(data, num_landmarks, motion_noise, measurement_noise)

    # Convert the result to visible data
    print('Results of SCALABLE GRAPH SLAM')
    for i in range(0, len(scalable_slam_mu), 2):
        index_x, index_y = i, i + 1
        if i < len(scalable_slam_mu) - num_landmarks * 2:
            print(f'Robot position:    [x={scalable_slam_mu[index_x]} y={scalable_slam_mu[index_y]}]')
        else:
            print(f'Landmark location: [x={scalable_slam_mu[index_x]} y={scalable_slam_mu[index_y]}]')

    # Plot the position of the real robot and landmarks, and the estimate position of the real robot ,
    # and landmarks. Also plot the path followed by the robot during the SLAM process, and the estimate
    # path of the robot
    plot_graph_slam_estimates(scalable_slam_mu, plot_data, num_landmarks)


if __name__ == '__main__':
    main()
