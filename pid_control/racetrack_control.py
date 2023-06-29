from math import *
from typing import List, Tuple
import matplotlib.pyplot as plt
from utils.robot import Robot


def run(params, radius, n: int, speed: float) -> Tuple[List[float], List[float], float]:
    """
    Performs a robot run of 2*n timesteps where the steering angle is given by a PID controller.
    The goal is to keep the robot as close as possible to the track
    Arguments:
        params: List containing the values for tau_p, tau_i and tau_d
        radius: The radius of the track that the robot is navigating
        n: number of steps to take by the robot
        speed: distance travelled per time step
    """
    # Initialize Robot instance
    robot = Robot(length=20.)
    robot.set(0.0, radius, pi / 2.0)
    err = 0.

    x_trajectory, y_trajectory = list(), list()

    prev_crosstrack_error = robot.cte(radius)    # Previous cte is set to current cte for the first timestep
    int_crosstrack_error = 0.0  # The integral value is initialized to 0

    # Run the robot motion using a PID controller based on the crosstrack error received by the cte method
    for i in range(n * 2):

        # cte is the current crosstrack error of the robot
        crosstrack_error = robot.cte(radius)

        # Calculate the differential part of the PID Controller as (y_t - y_t-1)
        diff_crosstrack_error = crosstrack_error - prev_crosstrack_error

        # Calculate the integral part of the PID Controller as (sum([y_0, y_1, y_2, .., y_t]))
        int_crosstrack_error += crosstrack_error

        # Calculate steering angle with PID controller and apply the movement on the robot
        steer = - params[0] * crosstrack_error \
                - params[2] * diff_crosstrack_error \
                - params[1] * int_crosstrack_error
        robot = robot.circular_move(steer, speed)

        # Save the new x and y coordinates after performing the motion
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        # Calculate error
        if i >= n:
            err += crosstrack_error ** 2

        # Set current cte to prev_cte
        prev_crosstrack_error = crosstrack_error

        print(f'STEP {i+1}        [x={robot.x} y={robot.y} orientation={robot.orientation}]')

    return x_trajectory, y_trajectory, err / float(n)


def main() -> None:
    radius = 25.0
    params = [10.0, 0., 15.0]
    speed = 1.
    n = 200
    x_trajectory, y_trajectory, err = run(params, radius, n, speed)

    print(f'\nError during run: {err}')
    plt.plot(x_trajectory, y_trajectory)
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.show()


if __name__ == '__main__':
    main()
