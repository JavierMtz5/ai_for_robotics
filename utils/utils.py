from math import *
from utils.robot import Robot
from typing import List, Tuple, Union, Callable, Optional, Dict
import numpy as np
import matplotlib.pyplot as plt
import time


def distance_between(point1: Union[Tuple[float, float], List[float]],
                     point2: Union[Tuple[float, float], List[float]]) -> float:
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_trunc(angle: float) -> float:
    """Maps all angles in radians to a domain of [-pi, pi]"""
    while angle < 0.0:
        angle += pi * 2
    return ((angle + pi) % (pi * 2)) - pi


def get_heading(hunter_position: Union[Tuple[float, float], List[float]],
                target_position: Union[Tuple[float, float], List[float]]) -> float:
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def test_pose_estimation(estimate_next_pos_fcn: Callable,
                         target_bot: Robot,
                         previous_iteration_data: Optional[List[List[float]]] = None) -> None:
    """
    Method used for testing and visualizing how well the function passed as parameter
    localizes the target robot
    Arguments:
        estimate_next_pos_fcn: Function which calculates the estimated next position for the target robot
        target_bot: Robot instance, which represents the target robot whose position must be predicted
        previous_iteration_data: List containing data about the previous iteration's measurement calculations.
            The format of this data is
            [[x0, y0, orient0, turning_angle0, distance0], [x1, y1, orient1, turning_angle1, distance1], ...]
    """
    localized = False
    distance = 1.5
    distance_tolerance = 0.01 * distance
    ctr = 0

    while not localized and ctr <= 10:
        ctr += 1
        measurement = target_bot.sense_robot_position()
        position_guess, previous_iteration_data = estimate_next_pos_fcn(measurement, previous_iteration_data)
        target_bot = target_bot.circular_move(steering=2 * pi / 10, distance=distance)
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print(f"Target Robot successfully localized after {ctr} steps")
            print(f'True position: {true_position} | Estimated position: {position_guess}')
            localized = True
        if ctr == 10:
            print("Could not locate the Target Robot in less than 10 timesteps")


def test_and_visualize_noisy_pose_estimation(estimate_next_pos_fcn: Callable,
                                             target_bot: Robot,
                                             previous_iteration_data: Optional[List[List[float]]] = None) -> None:
    localized = False
    distance = 1.5
    distance_tolerance = 0.01 * distance
    measurement_noise = 0.05 * distance
    ctr = 0

    # For Visualization
    import turtle  # You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    window.screensize(2000, 2000)
    size_multiplier = 25.0  # change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    # End of Visualization

    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense_robot_position()
        position_guess, previous_iteration_data = estimate_next_pos_fcn(measurement, measurement_noise,
                                                                        previous_iteration_data)
        target_bot = target_bot.circular_move(steering=2 * pi / 34., distance=distance)
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print(f"Target Robot successfully localized after {ctr} steps")
            print(f'True position: {true_position} | Estimated position: {position_guess}')
            localized = True
        if ctr == 1000:
            print("Could not locate the Target Robot in less than 1000 timesteps")

        # More Visualization
        measured_broken_robot.setheading(target_bot.orientation * 180 / pi)
        measured_broken_robot.goto(measurement[0] * size_multiplier, measurement[1] * size_multiplier - 200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.orientation * 180 / pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 200)
        broken_robot.stamp()
        prediction.setheading(target_bot.orientation * 180 / pi)
        prediction.goto(position_guess[0] * size_multiplier, position_guess[1] * size_multiplier - 200)
        prediction.stamp()
        # End of Visualization

    turtle.clearscreen()


def test_robot_capture(hunter_bot: Robot, target_bot: Robot,
                       next_move_fcn: Callable, max_distance_factor: float,
                       previous_iteration_data=None):
    """

    """
    distance = 1.5
    measurement_noise = 0.075
    max_distance = max_distance_factor * distance
    separation_tolerance = 0.02 * distance
    caught = False
    ctr = 0

    # For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    window.screensize(2000, 2000)
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0  # change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    # End of Visualization

    while not caught and ctr < 1000:

        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print(f"Target Robot successfully captured after {ctr} steps")
            caught = True

        target_measurement = target_bot.sense_robot_position()
        turning, hunter_distance, previous_iteration_data = next_move_fcn(hunter_position, hunter_bot.orientation,
                                                                          target_measurement, measurement_noise,
                                                                          max_distance, previous_iteration_data)
        if hunter_distance > max_distance:
            hunter_distance = max_distance
        hunter_bot = hunter_bot.circular_move(steering=turning, distance=hunter_distance)
        target_bot = target_bot.circular_move(steering=2 * pi / 30, distance=distance)

        # Visualize it
        measuredbroken_robot.setheading(target_bot.orientation * 180 / pi)
        measuredbroken_robot.goto(target_measurement[0] * size_multiplier,
                                  target_measurement[1] * size_multiplier - 100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.orientation * 180 / pi)
        broken_robot.goto(target_bot.x * size_multiplier, target_bot.y * size_multiplier - 100)
        chaser_robot.setheading(hunter_bot.orientation * 180 / pi)
        chaser_robot.goto(hunter_bot.x * size_multiplier, hunter_bot.y * size_multiplier - 100)
        # End of visualization

        time.sleep(2)

        ctr += 1
        if ctr >= 1000:
            print("Could not catch the Target Robot in less than 1000 timesteps")

    turtle.clearscreen()


def plot_graph_slam_estimates(estimate: np.ndarray, plot_data: Dict[str, list], num_landmarks: int) -> None:
    """
    Plots the real position of the landmarks and robot, and the estimate positions of the landmarks
    and the robot, predicted by the Graph SLAM algorithm
    Arguments:
        estimate: mu matrix obtained from the Graph SLAM algorithm.
        plot_data: dictionary containing two keys: landmarks and robot_positions. The first element contains the
            real position of the landmarks, and the second one contains the different positions reached by the robot
            during its motion.
        num_landmarks: number of landmarks
    """
    # Initialize plot and set axis limits
    plt.plot(0, 0)
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    # Plot the path followed by the robot along its motion, a   nd the estimate followed path
    plt.plot([robot_pos[0] for robot_pos in plot_data['robot_positions']],
             [robot_pos[1] for robot_pos in plot_data['robot_positions']])
    plt.plot([estimate[i*2] for i in range(len(estimate)//2 - num_landmarks)],
             [estimate[(i*2)+1] for i in range(len(estimate)//2 - num_landmarks)])
    # Plot the real positions of the landmarks, and the estimated positions
    plt.scatter(x=[landmark[0] for landmark in plot_data['landmarks']],
                y=[landmark[1] for landmark in plot_data['landmarks']], c='red')
    plt.scatter(x=[estimate[-1 * i] for i in range(num_landmarks * 2, 0, -2)],
                y=[estimate[-1 * i] for i in range((num_landmarks * 2) - 1, -1, -2)], c='green')
    # Plot the real position of the robot, and the estimated position
    plt.scatter(x=plot_data['robot_positions'][-1][0],
                y=plot_data['robot_positions'][-1][1], c='blue')
    plt.scatter(x=estimate[-2 + -1 * (num_landmarks * 2)],
                y=estimate[-1 + -1 * (num_landmarks * 2)], c='yellow')

    plt.show()
