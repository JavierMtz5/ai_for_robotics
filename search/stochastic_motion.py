# --------------
# USER INSTRUCTIONS
#
# Write a function called stochastic_value that
# returns two grids. The first grid, value, should
# contain the computed value of each cell as shown
# in the video. The second grid, policy, should
# contain the optimum policy for each cell.
#
# --------------
# GRADING NOTES
#
# We will be calling your stochastic_value function
# with several different grids and different values
# of success_prob, collision_cost, and cost_step.
# In order to be marked correct, your function must
# RETURN (it does not have to print) two grids,
# value and policy.
#
# When grading your value grid, we will compare the
# value of each cell with the true value according
# to this model. If your answer for each cell
# is sufficiently close to the correct answer
# (within 0.001), you will be marked as correct.

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']  # Use these when creating your policy grid.


# ---------------------------------------------
#  Modify the function stochastic_value below
# ---------------------------------------------

def stochastic_value(grid, goal, cost_step, collision_cost, success_prob):
    failure_prob = (1.0 - success_prob) / 2.0  # Probability(stepping left) = prob(stepping right) = failure_prob
    value = [[collision_cost for _ in range(len(grid[0]))] for _ in range(len(grid))]
    policy = [[' ' for _ in range(len(grid[0]))] for _ in range(len(grid))]

    change = True
    while change:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if x == goal[0] and y == goal[1]:
                    if value[x][y] > 0:
                        change = True
                        policy[x][y] = '*'
                        value[x][y] = 0

                elif grid[x][y] == 0:

                    for a in range(len(delta)):
                        x2_front = x + delta[a][0]
                        y2_front = y + delta[a][1]
                        x2_left = x + delta[(a - 1) % len(delta)][0]
                        x2_right = x + delta[(a + 1) % len(delta)][0]
                        y2_left = y + delta[(a - 1) % len(delta)][1]
                        y2_right = y + delta[(a + 1) % len(delta)][1]
                        v2 = 0
                        if len(grid) > x2_front >= 0 == grid[x2_front][y2_front] and 0 <= y2_front < len(grid[0]):

                            # Set value for stochastic front move
                            v2 += value[x2_front][y2_front] * success_prob + cost_step

                            # Check stochastic left move
                            if len(grid) > x2_left >= 0 == grid[x2_left][y2_left] and 0 <= y2_left < len(grid[0]):
                                v2 += value[x2_left][y2_left] * failure_prob
                            else:
                                v2 += collision_cost * failure_prob

                            # Check stochastic right move
                            if len(grid) > x2_right >= 0 == grid[x2_right][y2_right] and 0 <= y2_right < len(grid[0]):
                                v2 += value[x2_right][y2_right] * failure_prob
                            else:
                                v2 += collision_cost * failure_prob

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]

    return value, policy


# ---------------------------------------------
#  Use the code below to test your solution
# ---------------------------------------------

grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]
goal = [0, len(grid[0]) - 1]  # Goal is in top right corner
cost_step = 1
collision_cost = 1000
success_prob = 0.5

value, policy = stochastic_value(grid, goal, cost_step, collision_cost, success_prob)
for row in value:
    print(row)
for row in policy:
    print(row)

# Expected outputs:
#
# [471.9397246855924, 274.85364957758316, 161.5599867065471, 0],
# [334.05159958720344, 230.9574434590965, 183.69314862430264, 176.69517762501977],
# [398.3517867450282, 277.5898270101976, 246.09263437756917, 335.3944132514738],
# [700.1758933725141, 1000, 1000, 668.697206625737]


#
# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
