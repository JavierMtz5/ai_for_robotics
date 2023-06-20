from math import *

from robot import Robot


def main() -> None:
    world_size_data = 100.0
    landmarks_data = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]
    length = 20.

    myrobot = Robot(world_size_data, landmarks_data, length)
    myrobot.set(30.0, 20.0, 0.0)

    # 1) The following code should print [6.004885648174475, 3.7295952571373605, 1.9295669970654687, 0.8519663271732721]
    print(f'Robot 1:                    {[myrobot.x, myrobot.y, myrobot.orientation]}')
    print(f'Bearing Measurements 1:     {myrobot.sense_bearing()}', end='\n\n')

    myrobot = Robot(world_size_data, landmarks_data, length)
    myrobot.set(30.0, 20.0, pi / 5.0)

    # 2) The following code should print [5.376567117456516, 3.101276726419402, 1.3012484663475101, 0.22364779645531352]
    print(f'Robot 2:                    {[myrobot.x, myrobot.y, myrobot.orientation]}')
    print(f'Bearing Measurements 2:     {myrobot.sense_bearing()}')


if __name__ == '__main__':
    main()
