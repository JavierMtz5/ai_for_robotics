# Please only modify the indicated area below!

from math import *
import random

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class Robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        z = []
        for i in range(len(landmarks)):
            distance = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            distance += random.gauss(0.0, self.sense_noise)
            z.append(distance)
        return z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')

            # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        distance = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * distance)
        y = self.y + (sin(orientation) * distance)
        x %= world_size  # cyclic truncate
        y %= world_size

        # set particle
        res = Robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    @staticmethod
    def gaussian(mu, sigma, x):

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):

        # calculates how likely a measurement should be

        prob = 1.0
        for i in range(len(landmarks)):
            distance = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(distance, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def evaluate_particles(r, p):
    tot = 0.0
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
        dy = (p[i].y - r.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
        err = sqrt(dx * dx + dy * dy)
        tot += err
    return tot / float(len(p))


def main() -> None:

    robot = Robot()
    robot = robot.move(0.1, 5.0)
    n = 1000
    t = 10  # Leave this as 10 for grading purposes.

    p = []

    # Generate N particles and store them in list p
    for _ in range(n):
        r = Robot()
        r.set_noise(0.05, 0.05, 5.0)
        p.append(r)

    for t in range(t):
        robot = robot.move(0.1, 5.0)
        sensing = robot.sense()

        p2 = []
        for i in range(n):
            p2.append(p[i].move(0.1, 5.0))
        p = p2

        w = []
        for i in range(n):
            w.append(p[i].measurement_prob(sensing))

        p3 = []
        index = int(random.random() * n)
        beta = 0.0
        mw = max(w)
        for i in range(n):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % n
            p3.append(p[index])
        p = p3
        # enter code here, make sure that you output 10 print statements.
        print(evaluate_particles(robot, p))


if __name__ == '__main__':
    main()
