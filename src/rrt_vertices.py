#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import random


np.random.seed(444)


class RRT(GridMap):

    def __init__(self):
        super(RRT, self).__init__()
        self.max_velocity = 20
        #self.min_velocity = 0.001
        self.L = 0.04
        self.max_phi = np.pi/6
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
        # TODO: brac pod uwage kat i predkosc z 'closest'?
        u_s = random.random() * self.max_velocity
        # u_s = (random.random() * self.max_velocity + self.min_velocity)/(self.max_velocity + self.min_velocity)
        u_phi = (random.random() * 2 - 1) * self.max_phi
        return u_s, u_phi

    def check_path(self, point_closest, point_random):
        number_of_samples = 10
        best_point = (1e6, 1e6, 1e6)
        best_phi = 1e6
        best_s = 1e6
        for sample_iteration in range(number_of_samples):
            u_s, u_phi = self.random_u()
            valid = True
            for iteration in range(1, 101):
                x, y, theta = self.calc_point(point_closest, u_s/100 * iteration, u_phi)
                if self.is_free(x, y) and self.check_if_valid((x, y)):
                    continue
                else:
                    valid = False
            if valid:
                dist_rand = self.dist((x, y), (point_random[0], point_random[1]))
                dist_best = self.dist((best_point[0], best_point[1]), (point_random[0], point_random[1]))
                if(dist_rand < dist_best):
                    best_point = (x, y, theta)
                    best_s = u_s
                    best_phi = u_phi
        # TODO: przerobic, zeby nie zwracac None
        if best_phi != 1e6:
            return best_point, best_s, best_phi
        else:
            return (None, None, None), None, None

    # TODO: zmienic nazwy funkcji obliczajacych z check na cos innego
    def calc_point(self, point_closest, u_s, u_phi):
        theta_t = point_closest[2] + u_s / self.L * np.tan(u_phi) * self.delta_t
        x_t = point_closest[0] + u_s * np.cos(theta_t) * self.delta_t
        y_t = point_closest[1] + u_s * np.sin(theta_t) * self.delta_t
        return x_t, y_t, theta_t

    def future_check(self, point, u_s, u_phi):

        for iteration in range(1, 101):
            pass
        return True




    def search(self):

        # TODO: dodac predkosc samochodu
        #  w wierzcholku powinny byc wszystkie dane samochodu (predkosc i kat kol)
        self.parent[self.start] = None
        path = [self.end]
        is_no_path = True
        print("FDASLFSDAT")
        while is_no_path:
            random_point = self.random_point()
            closest = self.find_closest(random_point)
            point, u_s, u_phi = self.check_path(closest, random_point)
            # TODO: wyrzucanie punktow skierowanych w sciane
            if point != (None, None, None) and self.future_check(point, u_s, u_phi) == True:
                self.parent[(point[0], point[1], point[2])] = closest
                # print(u_phi)
            # TODO: publikowac wyszukiwanie jak trajektorie samochodu

            self.publish_search()



            # TODO: sprawdzanie celu i sciezka start -> cel
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
            rp.sleep(1)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
