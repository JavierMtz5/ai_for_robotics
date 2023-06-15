from math import *
from robot import Robot


def main() -> None:

    world_size_data = 100.0
    landmarks_data = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]

    # Initialize actual Robot for Test 1
    length = 20.
    myrobot = Robot(world_size_data, landmarks_data, length)
    myrobot.set(0.0, 0.0, 0.0)

    motions_1 = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]

    print(f'Robot 1:    {[myrobot.x, myrobot.y, myrobot.orientation]}')
    for motion in motions_1:
        steering, distance = motion
        myrobot.circular_move(steering, distance)
        print(f'Robot 1:    {[myrobot.x, myrobot.y, myrobot.orientation]}')
    print('\n')

    # Initialize actual Robot for Test 2
    length = 20.
    myrobot = Robot(world_size_data, landmarks_data, length)
    myrobot.set(0.0, 0.0, 0.0)

    motions_2 = [[0.2, 10.] for _ in range(10)]

    print(f'Robot 2:    {[myrobot.x, myrobot.y, myrobot.orientation]}')
    for motion in motions_2:
        steering, distance = motion
        myrobot.circular_move(steering, distance)
        print(f'Robot 2:    {[myrobot.x, myrobot.y, myrobot.orientation]}')


if __name__ == '__main__':
    main()
