import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt
from particle_filter.robot import Robot


def make_robot() -> Robot:
    """
    Creates a Robot instance with starting position (x=0, y=1, orientation=0), and
    sets the steering drift to 10 degrees
    """
    new_robot = Robot()
    new_robot.set(0.0, 1.0, 0.0)
    new_robot.set_steering_drift((10.0 * np.pi) / 180.0)
    return new_robot


def run(robot: Robot, params: List[float], n: int = 100, speed: float = 1.0
        ) -> Tuple[List[float], List[float], float]:
    """
    Performs a robot run of 2*n timesteps where the steering angle is given by a PID controller.
    The goal is to keep the robot as close as possible to y = 0, starting from y = 1 position
    Arguments:
        robot: Robot instance
        params: List containing the values for tau_p, tau_i and tau_d
        n: number of steps to take by the robot
        speed: distance travelled per time step
    """
    # Initialize the error to 0, and the lists to store the x and y coordinates visited by the robot
    x_trajectory, y_trajectory = list(), list()
    error = 0
    prev_cte = robot.y  # Previous y is set to current y for the first timestep
    int_cte = 0         # The integral value is initialized to 0

    for i in range(2 * n):

        # cte is the current position of the robot in the y axis
        cte = robot.y

        # Calculate the differential part of the PID Controller as (y_t - y_t-1)
        diff_cte = cte - prev_cte
        # Calculate the integral part of the PID Controller as (sum([y_0, y_1, y_2, .., y_t]))
        int_cte += cte

        prev_cte = cte
        # Calculate steering angle with PID controller and apply the movement on the robot
        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
        robot.circular_move(steer, speed)

        # Save the new x and y coordinates after performing the motion
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        if i >= n:
            error += cte ** 2

    return x_trajectory, y_trajectory, error / n


def twiddle(tol: float = 0.2, use_parameters: Tuple[bool, bool, bool] = (True, True, True)
            ) -> Tuple[List[float], float]:
    """
    Twiddle algorithm. Optimizes the values of the three parameters involved in the PID controller:
    tau_p, tau_i and tau_d.
    Arguments:
        tol: Tolerance. If the sum of the dp parameters is higher than tolerance,
        Twiddle algorithm is finished
        use_parameters: Tuple of boolean indicating whether to use each of the parameters
        in the optimization process. [tau_p, tau_i, tau_d]
    """
    # Don't forget to call `make_robot` before every call of `run`!
    # Initialize all parameters to 0 and the dp parameters to 1
    p = [0.0, 0.0, 0.0]
    dp = [1.0 if use_parameters[_] else 0. for _ in range(len(p))]

    # Create a robot and make a run to get the error
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    while sum(dp) > tol:

        for i in range(len(p)):

            # Only optimize the parameters given by the use_parameters method parameter
            if not use_parameters[i]:
                continue

            p[i] += dp[i]
            robot = make_robot()
            _, _, err = run(robot, p)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                _, _, err = run(robot, p)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9

    return p, best_err


def main() -> None:

    # Calculate the optimal parameters with Twiddle for PID Controller
    params, err = twiddle(0.2)
    print(f"Final twiddle error = {err}. Final parameters (tau_p, tau_i, tau_d): {params}")

    # Make a run with the optimal parameters for the PID controller
    robot = make_robot()
    x_trajectory, y_trajectory, err = run(robot, params)
    n = len(x_trajectory)

    fig, ax1 = plt.subplots(1, 1, figsize=(8, 8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    plt.show()


if __name__ == '__main__':
    main()
