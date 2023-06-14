from typing import List, Tuple


def optimum_policy(grid: List[List[int]], goal: List[int], cost: float,
                   possible_actions: List[List[int]], actions_symbols: List[str]
                   ) -> Tuple[List[List[str]], List[List[float]]]:
    """
    Applies Dynamic Programming to calculate the value of every state in the map.
    With this value matrix, the optimal policy for each state is also calculated.
    Arguments:
        grid: 2D grid, where cells with value 1 are walls and cells with 0 are navigable spaces
        goal: List of 2 integers representing the x and y positions of the goal state
        cost: float representing the cost of taking a step on the grid
        possible_actions: List containing the 4 possible movements to take: up, left, down and right
        actions_symbols: List containing the symbols for each of the possible movements, respectively
    """
    # Initialize a matrix to indicate the value of each grid cell (value)
    # and another matrix to indicate the actions taken on each state (policy)
    value = [[99 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    policy = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]

    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                # If x and y are goal state set value to 0 and set the symbol to *
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        change = True
                        value[x][y] = 0
                        policy[x][y] = '*'

                elif grid[x][y] == 0:

                    # If the cell is a navigable one, perform all the possible actions from that state
                    for a in range(len(possible_actions)):
                        x2 = x + possible_actions[a][0]
                        y2 = y + possible_actions[a][1]

                        # Check that the expanded state falls into the grid and that is a valid state
                        if (0 <= x2 < len(grid) and 0 <= y2 < len(grid[0])) and grid[x2][y2] == 0:
                            # Update new value for x and y state as the previous state value
                            # plus the cost of taking a step
                            v2 = value[x2][y2] + cost

                            # If the new value is lower than the previous one, update it and
                            # store the action taken to get to the new state
                            # (each cell should have the lowest possible value)
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = actions_symbols[a]

    return policy, value


def main():
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]]
    possible_actions = [[-1, 0],  # go up
                        [0, -1],  # go left
                        [1, 0],  # go down
                        [0, 1]]  # go right
    goal = [len(grid) - 1, len(grid[0]) - 1]
    cost = 1  # the cost associated with moving from a cell to an adjacent one
    action_symbols = ['^', '<', 'v', '>']

    policy, value = optimum_policy(grid, goal, cost, possible_actions, action_symbols)
    print(f'Policy: ')
    for row in policy:
        print(row)
    print('\n')

    print(f'Value Matrix: ')
    for row in value:
        print(row)


if __name__ == '__main__':
    main()
