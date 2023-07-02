# Graph SLAM

> This module contains four submodules: **graph_slam_2d**, **scalable_graph_slam** and **segmented_cte**

<details>
    <summary>Graph SLAM 2D</summary>

## Graph SLAM 2D

This module applies the Graph SLAM algorithm, which locates the Robot and 
the landmarks at the same time, thus being able to localize itself while creating a map. 

</details>

<details>
    <summary>Scalable Graph SLAM</summary>

## Scalable Graph SLAM

This module applies the Scalable Graph SLAM algorithm, which works in the same way as the Graph SLAM algorithm, 
but works with an Omega matrix which only needs to store the localization of the landmarks and the last position 
of the robot, instead of storing the position of the landmarks and the position of every state visited by the robot. 

</details>

<details>
    <summary>Segmented CTE</summary>

## Segmented CTE

This module creates an autonomous navigation system for the robot, following these steps:
1. Plan the optimal path to the goal state using A* algorithm.
2. Smooth the optimal path.
3. Initialize the real Robot, and the Particle Filter of 100 particles with the same position and orientation as the real robot, 
and same steering, distance and measurement noise.
4. The robot's steering angle on each timestep is given by a PID Controller which tries to minimize 
the crosstrack error, which is the distance from the robot to the closest point of the smoothed path. The distance 
from the robot to the track is measured considering the position estimated by the Particle Filter.
5. After the motion is applied, measurements are taken from the robot's new positions, and the particles of the 
Particle Filter are resampled.

In the following image the path followed by the Robot is shown in green, and the smoothed optimal path is shown in red.

<img src="../doc_images/slam/segmented_cte.png" alt="drawing" width="600" height="600"/>

</details>