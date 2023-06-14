from typing import List


def optimum_policy2D(
        grid: List[List[int]], init: List[int], goal: List[int], cost: List[float],
        possible_actions: List[int], actions_symbols: List[str], possible_orientation: List[List[int]]
) -> List[List[str]]:
    """
    Applies First Search algorithm to find the optimum path from the initial position to the goal state.
    Arguments:
        grid: 2D grid, where cells with value 1 are walls and cells with 0 are navigable spaces
        init: Initial state as 3 element List
        goal: List of 2 integers representing the x and y positions of the goal state
        cost: float representing the cost of taking a step on the grid
        possible_actions: List containing the 4 possible movements to take: up, left, down and right
        actions_symbols: List containing the symbols for each of the possible movements, respectively
        possible_orientation:
    """
    # Define matrix for storing the value of each cell in the grid, another matrix for storing the optimal
    # action for each 3D state, and another one for storing the optimal action in the 2D grid
    value = [[[99 for _ in range(len(grid[0]))] for _ in range(len(grid))],
             [[99 for _ in range(len(grid[0]))] for _ in range(len(grid))],
             [[99 for _ in range(len(grid[0]))] for _ in range(len(grid))],
             [[99 for _ in range(len(grid[0]))] for _ in range(len(grid))]]
    policy = [[[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))],
              [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))],
              [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))],
              [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]]
    policy2D = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]

    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):
                    # If x and y are goal state set value to 0 and set symbol to *
                    if x == goal[0] and y == goal[1]:
                        if value[orientation][x][y] > 0:
                            value[orientation][x][y] = 0
                            policy[orientation][x][y] = '*'
                            change = True

                    elif grid[x][y] == 0:
                        # If the cell is a navigable one, perform all the possible actions from that state
                        for a in range(len(possible_actions)):
                            orientation2 = (orientation + possible_actions[a]) % 4
                            x2 = x + possible_orientation[orientation2][0]
                            y2 = y + possible_orientation[orientation2][1]

                            # Check if the expanded state falls into the grid and that it is a valid state
                            if (0 <= x2 < len(grid) and 0 <= y2 < len(grid[0])) and grid[x2][y2] == 0:
                                # Update new value for x, y and orientation state as the previous state value
                                # plus the cost of taking a step
                                v2 = value[orientation2][x2][y2] + cost[a]

                                # If the new value is lower than the previous one, update it and
                                # store the action taken to get to the new state
                                # (each cell should have the lowest possible value)
                                if v2 < value[orientation][x][y]:
                                    change = True
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = actions_symbols[a]

    # Set the initial pose and transform to the 2D policy
    x, y, orientation = init
    policy2D[x][y] = policy[orientation][x][y]
    # Loop until the goal state is reached
    while policy[orientation][x][y] != '*':
        # If the policy was '#' then orientation remains the same
        if policy[orientation][x][y] == '#':
            orientation2 = orientation
        # If the policy was 'R' then orientation turns right (subtracting one index)
        elif policy[orientation][x][y] == 'R':
            orientation2 = (orientation - 1) % 4
        # If the policy was 'L' then orientation turns left (adding one index)
        elif policy[orientation][x][y] == 'L':
            orientation2 = (orientation + 1) % 4

        # Perform action with the optimal new orientation
        x += possible_orientation[orientation2][0]
        y += possible_orientation[orientation2][1]
        orientation = orientation2
        # Update 2D policy
        policy2D[x][y] = policy[orientation][x][y]

    return policy2D


def main():
    possible_orientations = [[-1, 0],  # go up
                             [0, -1],  # go left
                             [1, 0],  # go down
                             [0, 1]]  # go right
    grid = [[1, 1, 1, 0, 0, 0],
            [1, 1, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0],
            [1, 1, 1, 0, 1, 1],
            [1, 1, 1, 0, 1, 1]]
    possible_actions = [-1, 0, 1]
    actions_symbols = ['R', '#', 'L']
    initial_state = [4, 3, 0]  # [x, y, orientation]
    goal = [2, 0]
    cost = [2, 1, 20]

    policy2D = optimum_policy2D(grid, initial_state, goal, cost, possible_actions,
                                actions_symbols, possible_orientations)
    for row in policy2D:
        print(row)


if __name__ == '__main__':
    main()
