from typing import List, Union, Tuple
import numpy as np
from slam.graph_slam_2d import make_data, slam


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
    xi[0][0] += world_size / 2.
    xi[1][0] += world_size / 2.

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
            omega.value[0][0] += 1. / measurement_noise
            omega.value[index_L_x][index_L_x] += 1. / measurement_noise
            omega.value[0][index_L_x] += -1. / measurement_noise
            omega.value[index_L_x][0] += -1. / measurement_noise

            # Set measurement constraints on Xi for x-axis
            xi.value[0][0] += -1. * measure[1] * (1. / measurement_noise)
            xi.value[index_L_x][0] += measure[1] / measurement_noise

            # Set measurement constraints on Omega for y-axis
            omega.value[1][1] += 1. / measurement_noise
            omega.value[index_L_y][index_L_y] += 1. / measurement_noise
            omega.value[1][index_L_y] += -1. / measurement_noise
            omega.value[index_L_y][1] += -1. / measurement_noise

            # Set measurement constraints on Xi for y-axis
            xi.value[1][0] += -1. * measure[2] * (1. / measurement_noise)
            xi.value[index_L_y][0] += measure[2] / measurement_noise

        # Expand Omega and Xi to hold the constraints for the new motion
        lst = [_ for _ in range(omega.dimx + 2) if _ not in [2, 3]]
        omega = omega.expand(omega.dimx + 2, omega.dimy + 2, lst, lst)
        xi = xi.expand(xi.dimx + 2, xi.dimy, lst, [0])

        # Set motion constraint on Omega for x-axis
        omega.value[0][0] += 1. / motion_noise
        omega.value[2][0] += -1. / motion_noise
        omega.value[0][2] += -1. / motion_noise
        omega.value[2][2] += 1. / motion_noise

        # Set motion constraint on Xi for x-axis
        xi.value[0][0] += -1. * dx * (1. / motion_noise)
        xi.value[2][0] += dx / motion_noise

        # Set motion constraint on Omega for y-axis
        omega.value[1][1] += 1. / motion_noise
        omega.value[1][3] += -1. / motion_noise
        omega.value[3][3] += 1. / motion_noise
        omega.value[3][1] += -1. / motion_noise

        # Set motion constraint on Xi for y-axis
        xi.value[1][0] += -1. * dy * (1. / motion_noise)
        xi.value[3][0] += dy / motion_noise

        # Resize the matrix to allow rescaling
        omega_prima = omega.take([_ for _ in range(2, omega.dimx)])
        xi_prima = xi.take([_ for _ in range(2, omega.dimx)], [0])
        B = omega.take([0, 1])
        A = omega.take([0, 1], [_ for _ in range(2, omega.dimx)])
        C = xi.take([0, 1], [0])

        omega = omega_prima - A.transpose() * B.inverse() * A
        xi = xi_prima - A.transpose() * B.inverse() * C

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

    data = make_data(n, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)
    slam_mu = slam(data, n, num_landmarks, motion_noise, measurement_noise)

    # Convert the result to visible data
    for i in range(0, len(slam_mu), 2):
        index_x, index_y = i, i + 1
        if i < len(slam_mu) - num_landmarks * 2:
            print(f'[x={slam_mu[index_x]} y={slam_mu[index_y]}]')
        else:
            print(f'Landmark location: [x={slam_mu[index_x]} y={slam_mu[index_y]}]')

    data = make_data(n, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)
    scalable_slam_mu, scalable_slam_omega = online_slam(data, num_landmarks, motion_noise, measurement_noise)

    # Convert the result to visible data
    for i in range(0, len(scalable_slam_mu), 2):
        index_x, index_y = i, i + 1
        if i < len(scalable_slam_mu) - num_landmarks * 2:
            print(f'[x={scalable_slam_mu[index_x]} y={scalable_slam_mu[index_y]}]')
        else:
            print(f'Landmark location: [x={scalable_slam_mu[index_x]} y={scalable_slam_mu[index_y]}]')


if __name__ == '__main__':
    main()
