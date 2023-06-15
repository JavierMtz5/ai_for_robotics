from typing import List


def search(grid: List[List[int]], init: List[int], goal: List[int], cost: float,
           possible_actions: List[List[int]], action_symbols: List[str]) -> List[List[str]]:
    """
    Applies First Search algorithm to find the optimum path from the initial position to the goal state.
    Arguments:
        grid: 2D grid, where cells with value 1 are walls and cells with 0 are navigable spaces
        init: List of 2 integers representing the x and y positions of the initial state
        goal: List of 2 integers representing the x and y positions of the goal state
        cost: float representing the cost of taking a step on the grid
        possible_actions: List containing the 4 possible movements to take: up, left, down and right
        action_symbols: List containing the symbols for each of the possible movements, respectively
    """
    # Initialize a matrix to indicate which grid states are already closed,
    # and another matrix to indicate the actions taken on each state
    closed = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    actions = [[-1 for _ in range(len(grid[0]))] for _ in range(len(grid))]

    # Close the initial state in the closed matrix
    closed[init[0]][init[1]] = 1

    x, y = init
    g = 0

    # Initialize list to store all open nodes that must  be expanded
    open_nodes = [[g, x, y]]

    found = False
    resign = False

    while not found and not resign:
        # If solution was not found and there are no nodes to expand, exit the loop
        if len(open_nodes) == 0:
            resign = True

        else:
            open_nodes.sort()
            next_node = open_nodes.pop(0)
            g, x, y = next_node

            # If solution is found, exit the loop
            if x == goal[0] and y == goal[1]:
                found = True

            else:
                # Expand the 'next_node' node by applying all the possible actions from that state
                for i in range(len(possible_actions)):
                    x2 = x + possible_actions[i][0]
                    y2 = y + possible_actions[i][1]
                    # Check that the expanded state falls into the grid
                    if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                        # If the new state is not closed, add it to the open_nodes list, close the state,
                        # and store the action taken to get to that new state
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_nodes.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            actions[x2][y2] = i

    # Initialize grid on which the optimum path will be drawn
    policy = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]
    x, y = goal
    policy[x][y] = '*'  # Set symbol for goal state (*)
    # Loop until the initial state is reached
    while x != init[0] or y != init[1]:
        # Apply the inverse action to get the previous state following the optimum path
        x2 = x - possible_actions[actions[x][y]][0]
        y2 = y - possible_actions[actions[x][y]][1]
        # Set the symbol corresponding to the action taken in the new coordinates
        policy[x2][y2] = action_symbols[actions[x][y]]
        x, y = x2, y2

    return policy


def main() -> None:
    grid = [[0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 1, 0],
            [0, 0, 1, 0, 1, 0],
            [0, 0, 1, 0, 1, 0]]
    possible_actions = [[-1, 0],  # go up
                        [0, -1],  # go left
                        [1, 0],  # go down
                        [0, 1]]  # go right
    init = [0, 0]
    goal = [len(grid) - 1, len(grid[0]) - 1]
    cost = 1
    actions_symbols = ['^', '<', 'v', '>']

    expand = search(grid, init, goal, cost, possible_actions, actions_symbols)
    for row in expand:
        print(row)


if __name__ == '__main__':
    main()
