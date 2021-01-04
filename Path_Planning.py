"""

Path planning Sample Code with RRT with Reeds-Shepp path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import os
import random
import sys
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/PythonRobotics/ReedsSheppPath/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/PythonRobotics/RRTStar/")

try:
    import reeds_shepp_path_planning
    from rrt_star import RRTStar
except ImportError:
    raise

show_animation = True


class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=500,
                 connect_circle_dist=50.0
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist

        self.curvature = np.sqrt(2)
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def planning(self, animation=True, search_until_max_iter=True):
        """
        planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(new_node, self.obstacle_list):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     plt.plot(ox, oy, "ok", ms=30 * size)
        currentAxs = plt.gca()

        var = 0.447

        for (Ox, Oy, size_x, size_y) in self.obstacle_list:
            # Circles
            currentAxs.add_patch(Circle((Ox, Oy), var, fill=True, color='b')) #Bottom left
            currentAxs.add_patch(Circle((Ox+size_x, Oy), var, fill=True, color='b')) #Bottom right
            currentAxs.add_patch(Circle((Ox, Oy+size_y), var, fill=True,color='b')) #Top left
            currentAxs.add_patch(Circle((Ox+size_x, Oy+size_y), var, fill=True, color='b')) #Top right
            # Rectangles
            currentAxs.add_patch(Rectangle((Ox-var, Oy), size_x+2*var, size_y, fill=True, color='b'))
            currentAxs.add_patch(Rectangle((Ox, Oy-var), size_x, size_y+2*var, fill=True, color='b'))
            # Origineel
            currentAxs.add_patch(Rectangle((Ox, Oy), size_x, size_y, fill=True, color="k"))

        currentAxs.add_patch(Rectangle((0, 0), 45, 45, fill=False, color="k"))

        currentAxs.axis('equal')
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 46, -2, 46])
        # plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.0001)

    def plot_start_goal_arrow(self):
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        if not px:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                        random.uniform(self.min_rand, self.max_rand),
                        random.uniform(-math.pi, math.pi)
                        )

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path


# Change iteration amount here
def main(max_iter=500):
    print("Start " + __file__)

    # Uncomment Array to change scenario
    # Arrays are in form: [X, Y, X_size, Y_size]

    # Scenario 1
    obstacleList =  [[4,4,16,4],
                    [4,12,16,4],
                    [4,20,16,4],
                    [4,28,16,4],
                    [4,36,16,4],
                    [24,4,16,4],
                    [24,12,16,4],
                    [24,20,16,4],
                    [24,28,16,4],
                    [24,36,16,4]] 

    # # Scenario 2
    # obstacleList =  [[4,4,7,4],
    #                 [4,12,7,4],
    #                 [4,20,7,4],
    #                 [4,28,7,4],
    #                 [4,36,7,4],
    #                 [14,4,7,4],
    #                 [14,12,7,4],
    #                 [14,20,7,4],
    #                 [14,28,7,4],
    #                 [14,36,7,4],
    #                 [24,4,7,4],
    #                 [24,12,7,4],
    #                 [24,20,7,4],
    #                 [24,28,7,4],
    #                 [24,36,7,4],
    #                 [34,4,7,4],
    #                 [34,12,7,4],
    #                 [34,20,7,4],
    #                 [34,28,7,4],
    #                 [34,36,7,4]] 

    # Scenario 3
    # obstacleList =  [[4,4,12,4],
    #                 [4,20,12,4],
    #                 [4,36,12,4],
    #                 [24,5,4,35]]
    # Set Initial parameters
    start = [2.0, 2.0, np.deg2rad(90.0)]
    goal = [35.0, 25.5, np.deg2rad(0.0)]
    start_time = time.perf_counter()

    # Start actual algorithm
    # Colision detection can be found in PythonRobotics/RRT/rrt.py at line 214
    
    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             [0, 45.0], max_iter=max_iter)
    path = rrt_star_reeds_shepp.planning(animation=show_animation)

    # Calculate time needed
    end = time.perf_counter()
    print("time elapsed: ", end - start_time)
    # Draw final path
    if path and show_animation:  # pragma: no cover
        rrt_star_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        # plt.grid(True)
        ax = plt.gca()
        ax.axis('equal')
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()
