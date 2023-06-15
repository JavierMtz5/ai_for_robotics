from typing import List, Tuple


def stochastic_value(grid: List[List[int]], goal: List[int], cost_step: float,
                     collision_cost: float, success_prob: float, possible_actions: List[List[int]],
                     actions_symbols: List[str]) -> Tuple[List[List[float]], List[List[str]]]:
    """
    Applies Dynamic Programming to calculate the value of every state in the map.
    With this value matrix, the optimal policy for each state is also calculated.
    As the robot's stochastic motion implies that there is a certain probability of failure
    (the robot can move right or left instead of forward), the optimal policy is calculated taking
    into account the possible unexpected motions.
    Arguments:
        grid: 2D grid, where cells with value 1 are walls and cells with 0 are navigable spaces
        goal: List of 2 integers representing the x and y positions of the goal state
        cost_step: float representing the cost of taking a step on the grid
        collision_cost: float representing the cost of colliding
        success_prob: float between 0 and 1 representing the probability of performing a successful motion
        possible_actions: List containing the 4 possible movements to take: up, left, down and right
        actions_symbols: List containing the symbols for each of the possible movements, respectively
    """
    # Initialize the failure probability: p(left) = p(right) = p(failure)
    failure_prob = (1.0 - success_prob) / 2.0

    # Initialize value and policy matrices
    value = [[collision_cost for _ in range(len(grid[0]))] for _ in range(len(grid))]
    policy = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]

    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                # If x and y are goal state set value to 0 and set the symbol to *
                if x == goal[0] and y == goal[1]:
                    if value[x][y] > 0:
                        change = True
                        policy[x][y] = '*'
                        value[x][y] = 0

                elif grid[x][y] == 0:

                    # If the cell is a navigable one, perform all the possible actions from that state
                    for a in range(len(possible_actions)):
                        # Compute new x and y coordinates if movement is performed successfully
                        x2_front = x + possible_actions[a][0]
                        y2_front = y + possible_actions[a][1]

                        # Compute new x and y coordinates if movement fails (LEFT)
                        x2_left = x + possible_actions[(a - 1) % len(possible_actions)][0]
                        y2_left = y + possible_actions[(a - 1) % len(possible_actions)][1]

                        # Compute new x and y coordinates if movement fails (RIGHT)
                        x2_right = x + possible_actions[(a + 1) % len(possible_actions)][0]
                        y2_right = y + possible_actions[(a + 1) % len(possible_actions)][1]

                        v2 = 0
                        if (0 <= x2_front < len(grid) and
                                0 <= y2_front < len(grid[0])) and grid[x2_front][y2_front] == 0:

                            # Set value for stochastic front move
                            v2 += value[x2_front][y2_front] * success_prob + cost_step

                            # Check if stochastic left move falls inside the gird and its a
                            # valid position and calculate the value
                            if (0 <= x2_left < len(grid) and
                                    0 <= y2_left < len(grid[0])) and grid[x2_left][y2_left] == 0:
                                v2 += value[x2_left][y2_left] * failure_prob
                            # If not, compute the value as a collision
                            else:
                                v2 += collision_cost * failure_prob

                            # Check if stochastic right move falls inside the gird and its a
                            # valid position and calculate the value
                            if (0 <= x2_right < len(grid) and
                                    0 <= y2_right < len(grid[0])) and grid[x2_right][y2_right] == 0:
                                v2 += value[x2_right][y2_right] * failure_prob
                            # If not, compute the value as a collision
                            else:
                                v2 += collision_cost * failure_prob

                            # If the new value is lower than the previous one, update it and
                            # store the action taken to get to the new state
                            # (each cell should have the lowest possible value)
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = actions_symbols[a]

    return value, policy


def main() -> None:
    possible_actions = [[-1, 0],  # go up
                        [0, -1],  # go left
                        [1, 0],  # go down
                        [0, 1]]  # go right
    actions_symbols = ['^', '<', 'v', '>']
    grid = [[0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1, 1, 0]]
    goal = [0, len(grid[0]) - 1]
    cost_step = 1
    collision_cost = 1000
    success_prob = 0.5

    value, policy = stochastic_value(grid, goal, cost_step, collision_cost, success_prob,
                                     possible_actions, actions_symbols)
    print('Value Matrix: ')
    for row in value:
        print(row)
    print('\n')

    print('Policy: ')
    for row in policy:
        print(row)


if __name__ == '__main__':
    main()
