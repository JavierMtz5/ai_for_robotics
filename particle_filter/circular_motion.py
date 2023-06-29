from math import *
from utils.robot import Robot


def main() -> None:

    world_size_data = 100.0
    landmarks_data = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]

    # Initialize actual Robot for Test 1
    length = 20.
    robot = Robot(world_size_data, landmarks_data, length)
    robot.set(0.0, 0.0, 0.0)

    motions_1 = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]

    print(f'Robot 1:    [x={robot.x} y={robot.y} orientation={robot.orientation}]')
    for motion in motions_1:
        steering, distance = motion
        robot = robot.circular_move(steering, distance)
        print(f'Robot 1:    [x={robot.x} y={robot.y} orientation={robot.orientation}]')
    print('\n')

    # Initialize actual Robot for Test 2
    length = 20.
    robot = Robot(world_size_data, landmarks_data, length)
    robot.set(0.0, 0.0, 0.0)

    motions_2 = [[0.2, 10.] for _ in range(10)]

    print(f'Robot 2:    [x={robot.x} y={robot.y} orientation={robot.orientation}]')
    for motion in motions_2:
        steering, distance = motion
        robot = robot.circular_move(steering, distance)
        print(f'Robot 2:    [x={robot.x} y={robot.y} orientation={robot.orientation}]')


if __name__ == '__main__':
    main()
