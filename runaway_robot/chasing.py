from math import *
from typing import List
from particle_filter.robot import Robot
from main import estimate_next_pos_noisy


def next_move(hunter_position: List[float], hunter_heading: float,
              target_measurement: List[float], max_distance: float, OTHER=None):
    """
    Arguments:
        hunter_position: List of x and y coords of the hunter robot
        hunter_heading: float indicating the robot orientation in radians
        target_measurement: measurement received from the target robot
        max_distance: max distance to be travelled by the hunter robot
        OTHER: other
    """
    tau_p, tau_d = 2.0, 6.0
    if not OTHER:
        kalman_data = None
        target_prev_pos = hunter_position
        prev_cte = 0.
        OTHER = [None, None, None]

    else:
        kalman_data = OTHER[0]
        target_prev_pos = OTHER[1]
        prev_cte = OTHER[2]

    # First, the next position of the target robot is predicted

    target_next_pos, kalman_data = estimate_next_pos_noisy(target_measurement, measurement_noise, kalman_data)

    # The line between the previous target pos and the new target pos will be the
    # line to follow using a PID controller
    path = [target_prev_pos, target_next_pos]
    diff_cte = -prev_cte
    u, cte = Robot().segmented_cte(hunter_position[0], hunter_position[1],
                                   path, 0)
    diff_cte += cte

    # Calculate steering angle as a PID Controller
    turning = -tau_p * cte - tau_d * diff_cte
    distance = distance_between(hunter_position, target_next_pos)
    if distance > max_distance:
        distance = max_distance

    # Store data in OTHER for next iteration
    OTHER[0] = kalman_data
    OTHER[1] = target_next_pos
    OTHER[2] = cte

    return turning, distance, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    distance = 1.5
    max_distance = 1.94 * distance  # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        print(f'Hunter position: {hunter_position} | Target position: {target_position}')
        print(f'Distance from hunter robot to target robot: {separation}')
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense_robot_position()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.orientation, target_measurement,
                                                 max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot = hunter_bot.circular_move(steering=turning, distance=distance)

        # The target continues its (nearly) circular motion.
        target_bot = target_bot.circular_move(steering=2*pi / 30., distance=1.5)

        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER

measurement_noise = .05*1.5
target = Robot()
target.set(0.0, 10.0, 0.0)
target.set_noise(0., 0., 0., 0., 0., measurement_noise, 0.)

hunter = Robot()
hunter.set(-10.0, -10.0, 0.0)

print(demo_grading(hunter, target, next_move))
