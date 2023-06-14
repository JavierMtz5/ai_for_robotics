from typing import *


def localize(colors: List[List[str]], measurements: List[str],
             motions: List[List[int]], sensor_right: float, p_move: float) -> List[List[float]]:
    """
    Updates the grid of position probabilities after moving the robot and sensing its environment iteratively.
    Motion (y, x):
         [0,0] - don't move
         [0,1] - right
         [0,-1] - left
         [1,0] - down
         [-1,0] - up
    Arguments:
        colors: 2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
        measurements: list of measurements taken by the robot, each entry either 'R' or 'G'
        motions: list of actions taken by the robot, each entry of the form [dy,dx],
                 where dx refers to the change in the x-direction (positive meaning
                 movement to the right) and dy refers to the change in the y-direction
                 (positive meaning movement downward)
        sensor_right: float between 0 and 1, giving the probability that any given
                      measurement is correct; the probability that the measurement is
                      incorrect is 1-sensor_right
        p_move: float between 0 and 1, giving the probability that any given movement
                command takes place; the probability that the movement command fails
                (and the robot remains still) is 1-p_move
    """
    # Initializes probabilities to a uniform distribution
    p_init = 1.0 / float(len(colors)) / float(len(colors[0]))
    probabilities = [[p_init for _ in range(len(colors[0]))] for _ in range(len(colors))]

    for motion, measure in zip(motions, measurements):
        # Update grid of probabilities after moving
        probabilities = move(probabilities, p_move, motion)
        # Update grid of probabilities after sensing
        probabilities = sense(probabilities, sensor_right, colors, measure)

    return probabilities


def move(probabilities: List[List[float]], p_move: float, action: List[int]) -> List[List[float]]:
    """
    Updates the robot's position probabilities after moving. As movement is stochastic, there
    is a p_move probability that the robot will successfully perform the motion, and (1 - p_move)
    probability that it will not move
    """
    q = [[0. for _ in range(len(probabilities[0]))] for _ in range(len(probabilities))]
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            # Compute probability of reaching state after motion
            s = p_move * probabilities[(row - action[0]) % len(probabilities)][
                (col - action[1]) % len(probabilities[0])]
            # Compute probability of remaining still
            s += (1 - p_move) * probabilities[row][col]
            # Update state probability
            q[row][col] = s

    return q


def sense(probabilities: List[List[float]], sensor_right: float, colors: List[List[str]], sensing: str):
    """
    Updates the robot's position probabilities after sensing. As sensing is stochastic, there
    is a sensor_right probability that the robot will successfully sense the color, and (1 - sensor_right)
    probability that it will not sense it correctly
    """
    q = [[0. for _ in range(len(probabilities[0]))] for _ in range(len(probabilities))]
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            hit = sensing == colors[row][col]
            # Compute probability of being in the given state
            q[row][col] = probabilities[row][col] * (hit * sensor_right + (1 - hit) * (1 - sensor_right))

    # Normalize probabilities
    total_sum = sum([sum(lst) for lst in q])
    for row in range(len(probabilities)):
        for col in range(len(probabilities[0])):
            q[row][col] = q[row][col] / total_sum

    return q


def main() -> None:
    colors_grid = [['R', 'G', 'G', 'R', 'R'],
                   ['R', 'R', 'G', 'R', 'R'],
                   ['R', 'R', 'G', 'G', 'R'],
                   ['R', 'R', 'R', 'R', 'R']]
    measurements_seq = ['G', 'G', 'G', 'G', 'G']            # The robot sees a green color in every state
    motions_seq = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]  # Sequence of motions is > v v >
    probs = localize(colors_grid, measurements_seq, motions_seq, sensor_right=0.7, p_move=0.8)

    # Print final probabilities
    for prob_row in probs:
        print(prob_row)


if __name__ == '__main__':
    main()
