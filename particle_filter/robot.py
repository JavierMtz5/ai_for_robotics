import random
from typing import List, Union
from math import *


class Robot:
    """Robot class which represents a simulated robot"""
    def __init__(self, world_size: Union[float, None] = None,
                 landmarks: Union[List[List[float]], None] = None,
                 length: float = 5., measurement_range: float = -1.,
                 motion_noise: float = 0., measurement_noise: float = 0.):
        self.x = random.random() * world_size if world_size else 0.
        self.y = random.random() * world_size if world_size else 0.
        self.orientation = random.random() * 2.0 * pi
        self.length = length
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0
        self.bearing_noise = 0.0
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        self.measurement_noise = measurement_noise
        self.measurement_range = measurement_range
        self.motion_noise = motion_noise

        self.world_size = world_size
        self.landmarks = landmarks
        self.num_landmarks = len(self.landmarks) if self.landmarks else 0

    def make_landmarks(self, num_landmarks: int) -> None:
        """Creates random landmarks in the world"""
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks

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

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def sense_absolute_distance(self) -> List[float]:
        """
        Computes the distance between the different landmarks and the robot, and adds
        some gaussian noise (according to sense_noise attribute) to those distances to
        simulate the stochastic behavior of sensing.
        """
        z = list()
        for i in range(self.num_landmarks):
            # Compute distance from robot to landmark
            distance = sqrt((self.x - self.landmarks[i][0]) ** 2 + (self.y - self.landmarks[i][1]) ** 2)
            # Add sensing noise to the distance
            distance += random.gauss(0.0, self.sense_noise)
            z.append(distance)

        return z

    def sense_x_y(self) -> List[List[float]]:
        """
        Senses the x and y distance from the robot to the landmarks.
        If one of the distances are out of range the measurement is not included in the return
        """
        Z = []
        # For each landmark, calculate de x and y distance from the robot to the landmark
        # If the distance falls inside the measurement range, append dx, dy and the landmark index to Z
        for i in range(self.num_landmarks):
            dx = self.landmarks[i][0] - self.x + (random.random() * 2. - 1.) * self.measurement_noise
            dy = self.landmarks[i][1] - self.y + (random.random() * 2. - 1.) * self.measurement_noise
            if self.measurement_range < 0.0 or abs(dx) + abs(dy) <= self.measurement_range:
                Z.append([i, dx, dy])
        return Z

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

    def move(self, turn: float, forward: float, cyclic_world: bool = False) -> bool:
        """
        Performs the motion defined by the parameters on the current robot, and returns a new Robot
        instance with the new position and orientation reached.
        Rotation is first applied, and forward movement is then applied.
        If the motion leads to a valid new position the method returns True, otherwise returns False
        """
        if forward < 0:
            raise ValueError('Robot cant move backwards')

        # Turn and add turning noise
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # Move and add forwarding noise
        # TODO: self.motion_noise initialize to 0?
        distance = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * distance) + (random.random() * 2. - 1.) * self.motion_noise
        y = self.y + (sin(orientation) * distance) + (random.random() * 2. - 1.) * self.motion_noise

        if self.world_size and cyclic_world:
            x %= self.world_size
            y %= self.world_size

        if self.world_size and not cyclic_world:
            if 0. > x or x > self.world_size or 0. > y or y > self.world_size:
                return False

        self.x = x
        self.y = y
        self.orientation = orientation

        return True

    def circular_move(self, steering: float, distance: float,
                      tolerance: float = 0.001, max_steering_angle: float = pi/4.):
        """
        Performs the movement defined by the motion parameter, and returns a new Robot
        instance with the new position and orientation reached.
        Instead of a forward motion, this method allows for forward motion
        while keeping certain steering angle
        Arguments:
            steering: steering angle in radians
            distance: motion distance
            tolerance: if the turning angle beta is lower than this value,
            the motion is considered a forward motion
            max_steering_angle: maximum steering angle
        """
        # If steering or distance have invalid values, set valid values
        if steering > max_steering_angle:
            steering = max_steering_angle
        elif steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # Apply noise to the steering angle and the distance
        steering = random.gauss(steering, self.steering_noise)
        distance = random.gauss(distance, self.distance_noise)

        # Apply steering drift
        steering += self.steering_drift

        # Calculate beta: the turning angle
        beta = (distance * tan(steering)) / self.length

        # If beta is lower than 0.001, a forward motion is approximated
        if abs(beta) < tolerance:
            self.x += cos(self.orientation) * distance
            self.y += sin(self.orientation) * distance
            self.orientation = (self.orientation + beta) % (2 * pi)

        # Else, a circular motion is considered
        else:
            # Calculate the radius of the circular motion
            radius = distance / beta

            # Calculate the x and y coordinates of the center about which the robot rotates
            cx = self.x - sin(self.orientation) * radius
            cy = self.y + cos(self.orientation) * radius

            # Calculate new x, y and orientation
            self.orientation = (self.orientation + beta) % (2. * pi)
            self.x = cx + sin(self.orientation) * radius
            self.y = cy - cos(self.orientation) * radius

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

    def cte(self, radius: float) -> float:
        """
        Computes the crosstrack error of the robot.
        Arguments:
            radius: The radius of the track that the robot is navigating
        """
        cte = 0.
        # If the robot is moving in the left side of the track
        if self.x <= radius:
            cte = sqrt((self.x - radius) ** 2 + (self.y - radius) ** 2) - radius
        # If the robot is moving in the right side of the track
        elif self.x >= 3 * radius:
            cte = sqrt((self.x - 3 * radius) ** 2 + (self.y - radius) ** 2) - radius
        # If the robot is moving in the down side of the track, moving left
        elif 3 * radius > self.x >= radius > self.y:
            cte = -1 * self.y
        # If the robot is moving in the upper side of the track, moving right
        elif radius <= self.x < 3 * radius and self.y > radius:
            cte = self.y - 2 * radius

        return cte
