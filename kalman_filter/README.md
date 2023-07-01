# Localization with Kalman Filter

This program localizes/predicts the next state of the Robot via Kalman Filter. Kalman Filter is useful to 
reduce the uncertainty of both the robot's position and the robot's measurements. The uncertainty can be due to 
the robot drifting, having noise in the measurements, etc. Kalman Filtering estimates the next state based on 
past states/measurements, and a model of the robot's motion.

In this problem the robot is only able to measure its position. With its position and a model of the robot's motion 
the Kalman Filter is able to estimate the next position of the robot and the next velocity. The velocity of the 
robot is considered constant, so the model of the robot (the F transition matrix) is as follows:

![Alt text](../doc_images/transition_matrix.png?raw=true "Map")

##### Case 1

- Measurements: [1, 2, 3]

As velocity is constant, the Kalman Filter should predict that the next position is 4, and that the velocity is 1, as 
the robot moves one position per step.

##### Case 2

- Measurements: [1, 2, 3, 4, 5, 6, 7]

As velocity is constant, the Kalman Filter should predict that the next position is 8, and that the velocity is 1, as 
the robot moves one position per step. It is important to note that, as the Filter has more measurements than 
in Case 1, the Kalman Filter estimate will be more precise, which is visible in the lower values of the uncertainty 
covariance matrix P.

##### Case 3

- Measurements: [1, 3, 5, 7]

As velocity is constant, the Kalman Filter should predict that the next position is 9, and that the velocity is 2, as 
the robot moves two position per step.
