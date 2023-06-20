import copy
from typing import List
import matplotlib.pyplot as plt


def smooth(path: List[List[float]], fix: List[int], weight_data: float = 0.0,
           weight_smooth: float = 0.1, tolerance: float = 0.000001) -> List[List[float]]:
    """
    Smooths the given path considering that some of the path states are fixed.
    Arguments:
        path: List of 2 items Lists, where each of these List contain the x and y coordinate of each path state
        fix: List containing the index of the path states that must remain fixed
        weight_data: weigh for gradient descent
        weight_smooth: weigh for gradient descent
        tolerance: When the difference between the last smoothed value and the new one is less
            than tolerance, the smoothing process ends
    """
    newpath = copy.deepcopy(path)

    # When the difference between the last smoothed value and the new one is less
    # than tolerance, the smoothing process ends
    change = 1.
    while change > tolerance:
        change = 0.
        for i in range(len(path)):
            # If the path state is fixed, skip the smoothing step for this state
            if not fix[i]:
                # Update x and y coordinates separately
                for j in range(len(path[0])):
                    old_smoothed_value = newpath[i][j]
                    # Apply gradient descent to smooth the coordinate
                    newpath[i][j] += weight_data * (path[i][j] - newpath[i][j]) + \
                        weight_smooth * (newpath[(i + 1) % len(path)][j] + newpath[(i - 1) % len(path)][j] - 2 * newpath[i][j]) + \
                        0.5 * weight_smooth * (2. * newpath[(i - 1) % len(path)][j] - newpath[(i - 2) % len(path)][j] - newpath[i][j]) + \
                        0.5 * weight_smooth * (2. * newpath[(i + 1) % len(path)][j] - newpath[(i + 2) % len(path)][j] - newpath[i][j])
                    change += abs(old_smoothed_value - newpath[i][j])

    return newpath


def smooth_non_cyclic(path: List[List[float]], weight_data: float = 0.1,
                      weight_smooth: float = 0.1, tolerance: float = 0.000001) -> List[List[float]]:
    """
    """
    newpath = copy.deepcopy(path)

    # When the difference between the last smoothed value and the new one is less
    # than tolerance, the smoothing process ends
    change = tolerance
    while change >= tolerance:
        change = 0.
        for i in range(1, len(path) - 1):
            # Update x and y coordinates separately
            for j in range(len(path[0])):
                old_smoothed_value = newpath[i][j]
                # Apply gradient descent to smooth the coordinate
                newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                newpath[i][j] += weight_smooth * (newpath[i + 1][j] + newpath[i - 1][j] - (2. * newpath[i][j]))
                if i >= 2:
                    newpath[i][j] += 0.5 * weight_smooth * (2. * newpath[i - 1][j] - newpath[i - 2][j] - newpath[i][j])
                if i <= len(path) - 3:
                    newpath[i][j] += 0.5 * weight_smooth * (2. * newpath[i + 1][j] - newpath[i + 2][j] - newpath[i][j])
                change += abs(old_smoothed_value - newpath[i][j])

    return newpath


def main() -> None:
    paths = [
        [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [6, 1], [6, 2], [6, 3], [5, 3], [4, 3], [3, 3], [2, 3],
         [1, 3], [0, 3], [0, 2], [0, 1]],
        [[0, 0], [2, 0], [4, 0], [4, 2], [4, 4], [2, 4], [0, 4], [0, 2]]
    ]
    fixed_points = [[1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
                    [1, 0, 1, 0, 1, 0, 1, 0]]

    for path, fixed_point in zip(paths, fixed_points):
        smoothed_path = smooth(path, fixed_point)
        print('Original path: ')
        for i, coord in enumerate(path):
            x, y = coord
            print(f'x: {x}, y: {y} ', end='Fixed\n' if fixed_point[i] else '\n')
        print('\n')

        print('Smoothed path: ')
        for i, coord in enumerate(smoothed_path):
            x, y = coord
            print(f'x: {x}, y: {y} ', end='Fixed\n' if fixed_point[i] else '\n')

        # Plot the original and smoothed path for the constrained smoothing
        plt.plot([coord[0] for coord in path], [coord[1] for coord in path], 'g')
        plt.plot([coord[0] for coord in smoothed_path], [coord[1] for coord in smoothed_path], 'r')
        plt.show()
        print('\n\n===================================================================================')


if __name__ == '__main__':
    main()
