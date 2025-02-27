"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

show_animation = True

def line_intersect(Ax1, Ay1, Ax2, Ay2, Bx1, By1, Bx2, By2):
    d = (By2 - By1) * (Ax2 - Ax1) - (Bx2 - Bx1) * (Ay2 - Ay1)
    if d:
        uA = ((Bx2 - Bx1) * (Ay1 - By1) - (By2 - By1) * (Ax1 - Bx1)) / d
        uB = ((Ax2 - Ax1) * (Ay1 - By1) - (Ay2 - Ay1) * (Ax1 - Bx1)) / d
    else:
        return False
    if not(0 <= uA <= 1 and 0 <= uB <= 1):
        return False

def box_intersect(Ax1, Ay1, Ax2, Ay2, Ox, Oy, size_x, size_y):
    left = line_intersect(Ax1, Ay1, Ax2, Ay2, Ox, Oy, Ox, Oy+size_y)
    right = line_intersect(Ax1, Ay1, Ax2, Ay2, Ox+size_x, Oy, Ox, Oy+size_y)
    top = line_intersect(Ax1, Ay1, Ax2, Ay2, Ox, Oy+size_x, Ox+size_x, Oy+size_y)
    bottom = line_intersect(Ax1, Ay1, Ax2, Ay2, Ox, Oy, Ox+size_x, Oy)

    if left or right or top or bottom:
        return False #collision
    else:
        return True #safe


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)
        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     self.plot_circle(ox, oy, size)
        currentAxs = plt.gca()
        for object in self.obstacle_list:
            currentAxs.add_patch(Rectangle((object[0], object[1]), object[2], object[3], fill=True, alpha=1))

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        # plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):
        
        # Set radius of robot
        radius = 0.361 

        if node is None:
            return False

        # Check for the rectangles
        for (Ox, Oy, size_x, size_y) in obstacleList:
            for x in node.path_x:
                for y in node.path_y:
                    # Check Blue Rectangle
                    if x > Ox -radius and x < Ox + size_x + 2*radius and y > Oy and y < Oy + size_y:
                        return False
                    # Check Green Rectangle
                    elif x > Ox and x < Ox + size_x and y > Oy - radius and y < Oy + size_y + 2*radius:
                        return False
                    # Check for out of bounds condition
                    elif x > 45 - radius and x < radius and y > 45 - radius and y < radius:
                        return False
        
        # Check for corner circles
        for (ox, oy, size_x, size_y) in obstacleList:
            for i in range(4):
                if i == 0: # Bottom left
                    dx_list = [ox - x for x in node.path_x]
                    dy_list = [oy - y for y in node.path_y]
                elif i == 1: # Bottom right
                    dx_list = [ox + size_x - x for x in node.path_x]
                    dy_list = [oy - y for y in node.path_y]
                elif i == 2: # Top left
                    dx_list = [ox - x for x in node.path_x]
                    dy_list = [oy + size_y - y for y in node.path_y]
                elif i == 3: # Top right
                    dx_list = [ox + size_x - x for x in node.path_x]
                    dy_list = [oy + size_y - y for y in node.path_y]
            
                d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
                if min(d_list) <= radius**2:
                    return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=8, gy=7):
    print("start " + __file__)

    # ====Search Path with RRT====
    # obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
    #                  (9, 5, 2), (8, 10, 1)]  # [x, y, radius]

    obstacleList =  [[1,1,3,1], 
                    [1,3,3,1],
                    [1,5,3,1],
                    [1,7,3,1],
                    [1,9,3,1],
                    [6,1,3,1], 
                    [6,3,3,1],
                    [6,5,3,1],
                    [6,7,3,1],
                    [6,9,3,1]] # [X, Y, X_size, Y_size]

    # Set Initial parameters
    rrt = RRT(
        start=[0.5, 0.5],
        goal=[gx, gy],
        rand_area=[0, 11],
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
