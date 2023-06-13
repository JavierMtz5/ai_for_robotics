# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up

from typing import *


def localize(colors: List[List[str]], measurements: List[str],
             motions: List[List[int]], sensor_right: float, p_move: float) -> List[List[float]]:
    # Initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    probabilities = [[pinit for _ in range(len(colors[0]))] for _ in range(len(colors))]

    for motion, measure in zip(motions, measurements):
        probabilities = move(probabilities, p_move, motion)
        probabilities = sense(probabilities, sensor_right, colors, measure)

    return probabilities


def move(probabilities: List[List[float]], p_move: float, action: List[int]) -> List[List[float]]:
    q = [[0. for _ in range(len(probabilities[0]))] for _ in range(len(probabilities))]
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            s = p_move * probabilities[(row - action[0]) % len(probabilities)][
                (col - action[1]) % len(probabilities[0])]
            s += (1 - p_move) * probabilities[row][col]
            q[row][col] = s

    return q


def sense(probabilities: List[List[float]], sensor_right: float, colors: List[List[str]], sensing: str):
    q = [[0. for _ in range(len(probabilities[0]))] for _ in range(len(probabilities))]
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            hit = sensing == colors[row][col]
            q[row][col] = probabilities[row][col] * (hit * sensor_right + (1 - hit) * (1 - sensor_right))

    total_sum = sum([sum(lst) for lst in q])
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            q[row][col] = q[row][col] / total_sum

    return q


def show(probabilities: List[List[float]]):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x), r)) + ']' for r in probabilities]
    print('[' + ',\n '.join(rows) + ']')


#############################################################
# For the following test case, your output should be
# [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
#  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
#  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
#  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]
# (within a tolerance of +/- 0.001 for each entry)

colors_grid = [['R', 'G', 'G', 'R', 'R'],
               ['R', 'R', 'G', 'R', 'R'],
               ['R', 'R', 'G', 'G', 'R'],
               ['R', 'R', 'R', 'R', 'R']]
measurements_seq = ['G', 'G', 'G', 'G', 'G']            # The robot sees a green color in every state
motions_seq = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]  # Sequence of motions is > v v >
probs = localize(colors_grid, measurements_seq, motions_seq, sensor_right=0.7, p_move=0.8)
show(probs)
