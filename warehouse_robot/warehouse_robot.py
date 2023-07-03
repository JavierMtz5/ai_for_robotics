from typing import List, Union
import copy
import time

warehouse = [[1, 2, 3],
             [0, 0, 0],
             [0, 0, 0]]
dropzone = [2, 0]
todo = [2, 1]


def plan(warehouse: List[List[Union[int, str]]],
         dropzone: List[int],
         todo: List[int]):
    """

    """
    # Set dropzone cell to 0
    warehouse[dropzone[0]][dropzone[1]] = 0

    values, policies, tasks_cost = list(), list(), 0.
    possible_actions = [[-1, 0],  # up
                        [0, -1],  # left
                        [1, 0],  # down
                        [0, 1],  # right
                        [-1, 1],  # diagonal upper right
                        [-1, -1],  # diagonal upper left
                        [1, -1],  # diagonal lower left
                        [1, 1]]  # diagonal lower right
    # Cost of moving forward is 1, cost of moving diagonally is 1.5
    cost = [1, 1, 1, 1, 1.5, 1.5, 1.5, 1.5]
    action_symbols = ['^ ', '< ', 'v ', '> ', 'UR', 'UL', 'LL', 'LR']

    # Iterate through every task to perform
    for box in todo:

        box_pos = None
        value = [[99 for _ in range(len(warehouse[0]))] for _ in range(len(warehouse))]
        policy = [['  ' for _ in range(len(warehouse[0]))] for _ in range(len(warehouse))]

        change = True
        while change:
            change = False

            for x in range(len(warehouse)):
                for y in range(len(warehouse[0])):
                    # If x and y are the goal state set value to 0 and set the symbol to *
                    if warehouse[x][y] == box:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            policy[x][y] = '* '
                            box_pos = [x, y]
                            change = True

                    elif warehouse[x][y] == 0:

                        # If the cell is a navigable one, perform all the possible actions from that state
                        for a in range(len(possible_actions)):
                            x2 = x + possible_actions[a][0]
                            y2 = y + possible_actions[a][1]

                            # Check that the expanded state falls into the grid and that is a valid state
                            if (0 <= x2 < len(warehouse) and 0 <= y2 < len(warehouse[0])) and \
                                    (warehouse[x2][y2] == 0 or box_pos == [x2, y2]):
                                # Update new value for x and y state as the previous state value
                                # plus the cost of taking a step
                                v2 = value[x2][y2] + cost[a]

                                # If the new value is lower than the previous one, update it and
                                # store the action taken to get to the new state
                                # (each cell should have the lowest possible value)
                                if v2 < value[x][y]:
                                    value[x][y] = v2
                                    policy[x][y] = action_symbols[a]
                                    change = True

        values.append(value)
        policies.append(policy)

        print('Warehouse: ')
        for row in warehouse:
            print(row)
        print('\n')
        print('Policy: ')
        for row in policy:
            print(row)
        print('\n')
        print('Value Matrix:')
        for row in value:
            print(row)
        print('\n')

        tasks_cost += value[dropzone[0]][dropzone[1]] * 2.
        print('Cost of task: ', value[dropzone[0]][dropzone[1]] * 2., '\n\n')

        # Update warehouse setting the previous box cell to 0
        warehouse[box_pos[0]][box_pos[1]] = 0

    return tasks_cost


def solution_check(test, epsilon=0.00001):
    answer_list = []

    import time
    start = time.time()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print("\nTest case", i + 1, "passed!")
            answer_list.append(1)
            correct_answers += 1
        else:
            print("\nTest case ", i + 1, "unsuccessful. Your answer ", user_cost,
                  "was not within ", epsilon, "of ", true_cost)
            answer_list.append(0)
        print('========================================================================?)')
    runtime = time.time() - start
    if runtime > 1:
        print("Your code is too slow, try to optimize it! Running time was: ", runtime)
        return False
    if correct_answers == len(answer_list):
        print("\nYou passed all test cases!")
        return True
    else:
        print("\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!")
        return False


def main():
    # Test Case 1
    warehouse1 = [[1, 2, 3],
                  [0, 0, 0],
                  [0, 0, 0]]
    dropzone1 = [2, 0]
    todo1 = [2, 1]
    true_cost1 = 9
    # Test Case 2
    warehouse2 = [[1, 2, 3, 4],
                  [0, 0, 0, 0],
                  [5, 6, 7, 0],
                  ['x', 0, 0, 8]]
    dropzone2 = [3, 0]
    todo2 = [2, 5, 1]
    true_cost2 = 21

    # Test Case 3
    warehouse3 = [[1, 2, 3, 4, 5, 6, 7],
                  [0, 0, 0, 0, 0, 0, 0],
                  [8, 9, 10, 11, 0, 0, 0],
                  ['x', 0, 0, 0, 0, 0, 12]]
    dropzone3 = [3, 0]
    todo3 = [5, 10]
    true_cost3 = 18

    # Test Case 4
    warehouse4 = [[1, 17, 5, 18, 9, 19, 13],
                  [2, 0, 6, 0, 10, 0, 14],
                  [3, 0, 7, 0, 11, 0, 15],
                  [4, 0, 8, 0, 12, 0, 16],
                  [0, 0, 0, 0, 0, 0, 'x']]
    dropzone4 = [4, 6]
    todo4 = [13, 11, 6, 17]
    true_cost4 = 41

    testing_suite = [[warehouse1, warehouse2, warehouse3, warehouse4],
                     [dropzone1, dropzone2, dropzone3, dropzone4],
                     [todo1, todo2, todo3, todo4],
                     [true_cost1, true_cost2, true_cost3, true_cost4]]

    solution_check(testing_suite)


if __name__ == '__main__':
    main()
