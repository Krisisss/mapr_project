#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.1

    def check_if_valid(self, a, b):
        if a[0] < 0 or a[0] > self.width or a[1] < 0 or a[1] > self.height:
            return False
        if b[0] < 0 or b[0] > self.width or b[1] < 0 or b[1] > self.height:
            return False
        for i in range(101):
            x, y = self.pt_in_dir(a, b, i/100.0)
            x = int(x * 50)
            y = int(y * 50)
            if self.map[y][x] > 50:
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

    def new_pt(self, pt, closest):
        point = self.pt_in_dir(closest, pt, self.step/self.dist(pt, closest))
        return point

    def pt_in_dir(self, st_point, end_point, percent):
        x = percent*(end_point[0] - st_point[0])+st_point[0]
        y = percent*(end_point[1] - st_point[1])+st_point[1]
        return [x, y]

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
            point = self.new_pt(point, closest)
            if self.check_if_valid(point, closest):
                self.parent[(point[0], point[1])] = closest
            if self.check_if_valid(point, self.end):
                last = (point[0], point[1])
                path.append(last)
                while is_no_path:
                    last = self.parent[last]
                    path.append(last)
                    if last == self.start:
                        is_no_path = False

            self.publish_search()
            #rp.sleep(0.05)

        self.publish_path(path)
        print('path is finding')
        while not rp.is_shutdown():
            rp.sleep(0.01)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
