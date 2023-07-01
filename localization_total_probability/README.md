# Localization with Total Probability

This program shows how to locate a robot in a discrete environment. Since at the beginning the robot does not know where 
it is, has not moved and has not taken any measurements of the environment, it will start with a uniform probability 
distribution, so that the robot has the same probability of being in any cell of the map. 

When the robot moves, since there is a chance of failure when performing the motion, more uncertainty is added to the robot's position. 
For example, if the robot is located in a cell with a probability of 100%, and moves one cell to the right with a 
probability of success of 80%, then the probability that the robot is still in the same cell as before (because motion failed) is 20%, and 
the probability that it has moved to the next cell is 80%. It can be seen how before the movement the position of 
the robot was defined, while after the movement more uncertainty has been added to its position.

On the other hand, when the robot takes measurements of the environment, the uncertainty is reduced, and the robot 
is closer to being able to locate itself correctly. While the measurements also have noise that can return incorrect 
values, as long as the noise is low enough, the uncertainty of the robot's localization will decrease with each measurement.

In this program a map is created with cells painted green or red, as shown in the image. 

![Alt text](../doc_images/localization_prob_grid.png?raw=true "Map")

The robot performs 5 movements, and between each of these movements it takes measurements of the color it sees in 
the new cell it has reached. The sequence of movements and measurements is as follows:

![Alt text](../doc_images/motion_and_sensing_localization.png?raw=true "Motions and Measurements")

The goal of the program is to modify the probabilities of being in each cell as the robot moves and takes 
measurements. In the end, the cell with the highest probability should correctly indicate the final position of the robot. 

In this case the program is executed such that the robot has a probability of motion success of 80%, and a 
probability that the sensor detects the color correctly of 70%.

The final probabilities after performing the mentioned motions and measurements are:

![Alt text](../doc_images/localization_probabilities.png?raw=true "Localization")

As can be seen, the cell with the highest probability is the cell in which the robot is positioned, according to 
the movements and measurements taken during the path. However, the probability of being in this state is not 100%, 
due to the probability of failure during movements and measurements.