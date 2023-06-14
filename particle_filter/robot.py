import random
from typing import List
from math import *


class Robot:
    """Robot class which represents a simulated robot"""
    def __init__(self, world_size: float, landmarks: List[List[float]], length: float = 5.):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.length = length
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0
        self.bearing_noise = 0.0
        self.steering_noise = 0.0
        self.distance_noise = 0.0

        self.world_size = world_size
        self.landmarks = landmarks

    def set(self, new_x: float, new_y: float, new_orientation: float) -> None:
        """Allows to set the new position (x, y) of the robot as well as its new orientation"""
        # Perform checking for new position/orientation
        if 0 > new_x >= self.world_size:
            raise ValueError('X coordinate out of bound')
        if 0 > new_y >= self.world_size:
            raise ValueError('Y coordinate out of bound')
        if 0 > new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        # Set new position and orientation
        self.x = new_x
        self.y = new_y
        self.orientation = new_orientation

    def set_noise(self, forward_noise: float, turn_noise: float, sensing_noise: float,
                  bearing_noise: float, steering_noise: float, distance_noise) -> None:
        """Setter for the forward, turn and sensing noise of the robot"""
        self.forward_noise = forward_noise
        self.turn_noise = turn_noise
        self.sense_noise = sensing_noise
        self.bearing_noise = bearing_noise
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def sense(self) -> List[float]:
        """
        Computes the distance between the different landmarks and the robot, and adds
        some gaussian noise (according to sense_noise attribute) to those distances to
        simulate the stochastic behavior of sensing.
        """
        z = list()
        for i in range(len(self.landmarks)):
            # Compute distance from robot to landmark
            distance = sqrt((self.x - self.landmarks[i][0]) ** 2 + (self.y - self.landmarks[i][1]) ** 2)
            # Add sensing noise to the distance
            distance += random.gauss(0.0, self.sense_noise)
            z.append(distance)

        return z

    def sense_bearing(self):
        """Returns the bearings of the robot to each of the landmarks"""
        z = list()
        for landmark in self.landmarks:
            # Calculate bearing/angle from robot's orientation to the landmarks
            dx, dy = landmark[1] - self.x, landmark[0] - self.y
            bearing = atan2(dy, dx) - self.orientation
            bearing %= 2 * pi
            z.append(bearing)

        return z

    def move(self, turn: float, forward: float):
        """
        Performs the motion defined by the parameters on the current robot, and returns a new Robot
        instance with the new position and orientation reached. Rotation is first applied, and forward
        movement is then included.
        """
        if forward < 0:
            raise ValueError('Robot cant move backwards')

        # Turn and add turning noise
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # Move and add forwarding noise
        distance = float(forward) + random.gauss(0.0, self.forward_noise)
        x = (self.x + (cos(orientation) * distance)) % self.world_size
        y = (self.y + (sin(orientation) * distance)) % self.world_size

        # Create new Robot instance with the noisy position and orientation reached after the move
        robot = Robot(self.world_size, self.landmarks)
        robot.set(x, y, orientation)
        robot.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,
                        self.bearing_noise, self.steering_noise, self.distance_noise)

        return robot

    def circular_move(self, motion: List[float]):
        """
        Performs the movement defined by the motion parameter, and returns a new Robot
        instance with the new position and orientation reached. Instead of a forward motion,
        this method allows for forward motion while keeping certain steering angle
        Arguments:
            motion: list containing the motion [steering_angle, distance]
        """
        # Calculate beta: the turning angle
        beta = motion[1] * tan(motion[0]) / self.length

        # If beta is lower than 0.001, a forward motion is considered
        if abs(beta) < 0.001:
            x = self.x + (cos(self.orientation) * motion[1])
            y = self.y + (sin(self.orientation) * motion[1])

        # Else, a circular motion is considered
        else:
            # Calculate the radius of the circular motion
            radius = motion[1] / beta

            # Calculate the x and y coordinates of the center about which the robot rotates
            cx = self.x - sin(self.orientation) * radius
            cy = self.y + cos(self.orientation) * radius

            # Calculate new x and y coordinates
            x = cx + sin(self.orientation + beta) * radius
            y = cy - cos(self.orientation + beta) * radius

        orientation = self.orientation + beta
        orientation %= 2 * pi

        robot = Robot(self.world_size, self.landmarks, self.length)
        robot.set(x, y, orientation)
        robot.set_noise(self.forward_noise, self.turn_noise, self.sense_noise,
                        self.bearing_noise, self.steering_noise, self.distance_noise)

        return robot

    @staticmethod
    def gaussian(mu: float, sigma: float, val: float) -> float:
        """Calculates the probability of the value parameter for 1-dim Gaussian with mean mu and variance sigma"""
        return exp(- ((mu - val) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        """
        Compares the sensing of the robot of the landmarks and the actual distance from the robot
        to the landmarks, and computes how probable is for the robot to be in the correct position.
        Arguments:
            measurement: sensing of the actual robot.
        """
        prob = 1.0
        for i in range(len(self.landmarks)):
            # Compute distance between particle/robot and the landmarks
            distance = sqrt((self.x - self.landmarks[i][0]) ** 2 + (self.y - self.landmarks[i][1]) ** 2)
            # Compute probability of being in certain state and sensing those landmark measurements
            prob *= self.gaussian(distance, self.sense_noise, measurement[i])

        return prob
