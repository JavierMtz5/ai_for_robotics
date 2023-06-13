# ----------
# User Instructions:
#
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1, 0],  # go up
           [0, -1],  # go left
           [1, 0],  # go down
           [0, 1]]  # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0]  # given in the form [row,col,direction]
# direction = 0: up
#             1: left
#             2: down
#             3: right

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 20]  # cost has 3 values, corresponding to making


# a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D(grid, init, goal, cost):
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

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):
                    if x == goal[0] and y == goal:
                        value[orientation][x][y] = 0
                        policy[orientation][x][y] = '*'
                        change = True

                    elif grid[x][y] == 0:
                        for a in range(len(action)):
                            orientation2 = (orientation + action[a]) % 4
                            x2 = x + forward[orientation2][0]
                            y2 = y + forward[orientation2][1]

                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                v2 = value[orientation][x][y] + cost[a]
                                if v2 < value[orientation][x][y]:
                                    change = True
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[a]

    x, y, orientation = init[0], init[1], init[2]
    policy2D[x][y] = policy[orientation][x][y]
    while policy[orientation][x][y] != '*':
        if policy[orientation][x][y] == '#':
            orientation2 = orientation
        elif policy[orientation][x][y] == 'R':
            orientation2 = (orientation - 1) % 4
        elif policy[orientation][x][y] == 'L':
            orientation2 = (orientation + 1) % 4

        x += forward[orientation2][0]
        y += forward[orientation2][1]
        orientation = orientation2
        policy2D[x][y] = policy[orientation][x][y]

    return policy2D


policy2D = optimum_policy2D(grid, init, goal, cost)
print(policy2D)
