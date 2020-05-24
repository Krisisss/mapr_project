#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import random


np.random.seed(444)


class RRT(GridMap):

    def __init__(self):
        super(RRT, self).__init__()
        self.max_velocity = 50
        self.min_velocity = 1
        self.L = 0.04
        self.max_theta = np.pi/6
        self.delta_t = 0.01 # 10 ms
        self.step = 0.1

    def check_if_valid(self, a):
        if a[0] < 0 or a[0] > self.width or a[1] < 0 or a[1] > self.height:
            return False
        return True

    def is_free(self, x, y):
        x = int(x*50)
        y = int(y*50)
        return self.map[y][x] < 100

    def dist(self, pos1, pos2):
        distance = ((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)**0.5
        return distance

    def random_point(self):
        x = np.random.random_sample()*self.width
        y = np.random.random_sample()*self.height
        return np.array([x, y])

    def find_closest(self, pos):
        min = 1e6
        for point in self.parent:
            if self.dist(pos, point) < min:
                min = self.dist(pos, point)
                closest = point
        return closest

    def random_u(self):
        u_s = random.random() * self.max_velocity
        u_phi = (random.random() * 2 - 1) * self.max_theta
        return u_s, u_phi

    def check_path(self, point_closest):
        u_s, u_phi = self.random_u()
        for iteration in range(1, 101):
            free, x, y, theta = self.check_point(point_closest, u_s/100 * iteration, u_phi)
            if free and self.check_if_valid((x, y)):
                continue
            else:
                return None, None, None
        return (x, y, theta), u_s, u_phi

    def check_point(self, point_closest, u_s, u_phi):
        theta_t = point_closest[2] + u_s / self.L * np.tan(u_phi) * self.delta_t
        x_t = point_closest[0] + u_s * np.cos(theta_t) * self.delta_t
        y_t = point_closest[1] + u_s * np.sin(theta_t) * self.delta_t
        free = self.is_free(x_t, y_t)
        return free, x_t, y_t, theta_t


    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None
        path = [self.end]
        is_no_path = True
        while is_no_path:
            point = self.random_point()
            closest = self.find_closest(point)
            point, u_s, u_phi = self.check_path(closest)
            if u_s is None:
                continue
            self.parent[(point[0], point[1], point[2])] = closest
            print(point)
            self.publish_search()


            # if self.check_if_valid(point, self.end):
            #     last = (point[0], point[1])
            #     path.append(last)
            #     while is_no_path:
            #         last = self.parent[last]
            #         path.append(last)
            #         if last == self.start:
            #             is_no_path = False

            #rp.sleep(0.05)

        self.publish_path(path)
        print('path is finding')
        while not rp.is_shutdown():
            rp.sleep(0.01)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
