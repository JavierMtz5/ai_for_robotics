# Particle Filter

> This module contains three submodules: **circular_motion**, **particle_filter_sensing** and **particle_filters**

## Circular Motion

This module is used to test that the **circular_motion** method of the **Robot** class performs the circular 
movement of the robot correctly.

- The motion of the Robot, which in this case has 4 wheels, is approximated to the motion of a bicycle with the same 
length between the front and back wheels as the robot. 
- The state of the robot is defined by its x and y coordinates, 
and by the robot's orientation, which is the angle between the robot's orientation and the x-axis. 
- The motion depends on the distance that the robot travels per timestep, and the steering angle of the front 
wheels, which is the angle between the orientation of the front wheels and the orientation of the robot.

![Alt text](../doc_images/robot_circular_motion.png?raw=true "Map")

The equations and diagram used to calculate the position of the robot in the next timestep are shown in 
the following image:

![Alt text](../doc_images/robot_circular_motion_eq.png?raw=true "Map")

If the turning angle 𝛽 is very low the motion can be considered rectilinear, in which case the following equations 
are used for calculating the next position and orientation of the robot:

```math
x' = x + cos(\theta) * d
y' = y + sin(\theta) * d
\theta' = (\theta + \beta) mod 2\pi
```

## Particle Filter Sensing

This module is used to test that the **sense_bearing** method of the **Robot** class correctly outputs the angles 
between the robot's orientation and the position of the landmarks.

![Alt text](../doc_images/robot_sense_bearing.png?raw=true "Map")

## Particle Filter


