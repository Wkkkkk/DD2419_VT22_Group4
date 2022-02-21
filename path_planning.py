import json
from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import mpl_toolkits.mplot3d as a3


class Node:
    def __init__(self, index, parent=None, position=np.zeros(3), yaw=0):
        self.index = index
        self.position = position
        self.yaw = yaw
        self.parent = parent
        self.children = []
        self.cost2go = 0
        self.cost2come = 0
        self.cost = 0


class Planning:
    def __init__(self, root, goal, grid):
        self.root = root
        self.goal = goal
        grid[root.index] = root
        grid[goal.index] = goal

        self.grid = grid

        X = grid.dim[0]
        Y = grid.dim[1]
        self.neighbours = lambda x, y: [grid[(i, j)] for i in range(x-1, x+2)
                                         for j in range(y-1, y+2)
                                         if ((0 <= i <= X) and (0 <= j <= Y) and i != x and j != y
                                             and grid[(i, j)] != grid.occupied_space)]

    def compute_cost(self, node):
        if node.parent is self.root:
            node.cost2come = np.linalg.norm(node.position - node.parent.position)
        else:
            node.cost2come = node.parent.cost2come + np.linalg.norm(node.position-node.parent.position)

        H_path = self.grid.trajectory(node.index, self.goal.index, True)

        prev_cell = node.position
        for cell in H_path[1:]:
            pos2D = self.grid.index_to_world(cell)
            cell_pos = np.array([pos2D[0], pos2D[1], self.root.position[2]])
            node.cost2go += np.linalg.norm(prev_cell-cell_pos)
            prev_cell = cell_pos

        node.cost = node.cost2go + node.cost2come

    def run(self):
        goal_found = False
        open_set = []
        closed_set = set()

        open_set.append(self.root)

        while len(open_set) > 0:
            node = open_set[0]
            for e in open_set:
                if e.cost <= node.cost:
                    if e.cost2go < node.cost2go:
                        node = e

            open_set.remove(node)
            closed_set.add(node)

            if node is self.goal:
                goal_found = True
                break

            for neighbour in self.neighbours(node.index[0], node.index[1]):
                if neighbour in closed_set:
                    continue

                cost2neighbour = node.cost2come + np.linalg.norm(node.position-neighbour.position)
                if cost2neighbour < neighbour.cost2come or neighbour not in open_set:
                    neighbour.parent = node
                    self.compute_cost(neighbour)

                    if neighbour not in open_set:
                        open_set.append(neighbour)

        if goal_found:
            node = self.goal
            while node is not self.root:
                q = node.parent
                while q is not self.root:
                    q = q.parent
                    if not self.grid.trajectory(node.index, q.index, False):
                        node.parent = q
                    else:
                        break
                p = node.parent
                p.yaw = atan2(node.position[1] - p.position[1], node.position[0] - p.position[0])
                node = p
        else:
            print("Could not find a path to the goal!")


class GridMap:
    def __init__(self, bounds, radius, resolution, obstacles, root_z):
        self.occupied_space = 1
        self.radius = radius
        self.bounds = bounds
        self.resolution = resolution

        X = int((self.bounds[1][0] - self.bounds[0][0]) / self.resolution) + 1
        Y = int((self.bounds[1][1] - self.bounds[0][1]) / self.resolution) + 1
        self.dim = [X - 1, Y - 1]
        self.map = self.create_map(obstacles, root_z)

    def __getitem__(self, index):
        x, y = index
        return self.map[x][y]

    def __setitem__(self, index, val):
        x, y = index
        self.map[x][y] = val

    def convert_to_index(self, pos):
        float_index = (pos[0:2] - self.bounds[0][0:2])/self.resolution
        return float_index.astype(int)

    def create_map(self, obstacles, root_z):
        map = np.empty((self.dim[0]+1, self.dim[1]+1), dtype=object)

        for obs in obstacles:
            v1 = np.array(obs["start"])
            v2 = np.array(obs["stop"])
            sign1 = np.ones(3)
            sign2 = np.ones(3)
            if v1[0] > v2[0]:
                sign2[0] = -1
            else:
                sign1[0] = -1
            if v1[1] > v2[1]:
                sign2[1] = -1
            else:
                sign1[1] = -1

            index1 = self.convert_to_index(v1+self.radius*sign1)
            index2 = self.convert_to_index(v2+self.radius*sign2)
            if index1[0] > index2[0]:
                min_x = index2[0]
                max_x = index1[0]
            else:
                min_x = index1[0]
                max_x = index2[0]
            if index1[1] > index2[1]:
                min_y = index2[1]
                max_y = index1[1]
            else:
                min_y = index1[1]
                max_y = index2[1]

            for x in range(min_x, max_x+1):
                for y in range(min_y, max_y+1):
                    map[x][y] = self.occupied_space

        for x in range(0, self.dim[0]+1):
            for y in range(0, self.dim[1]+1):
                if map[x][y] != self.occupied_space:
                    index = np.array([x, y])
                    pos2D = self.index_to_world(index)
                    map[x][y] = Node(index, None, np.array([pos2D[0], pos2D[1], root_z]))

        return map

    def occupied_cell(self, index):
        if self.map[index[0]][index[1]] == self.occupied_space:
            return True
        else:
            return False

    def trajectory(self, start, end, returnList):
        """ Bresenham's line algorithm """
        x_start, y_start = start
        x_end, y_end = end
        (dx, dy) = (fabs(x_end - x_start), fabs(y_end - y_start))
        if x_end > x_start:
            x_inc = 1
        else:
            x_inc = -1
        if y_end > y_start:
            y_inc = 1
        else:
            y_inc = -1

        traversed = []
        if dx >= dy:
            p = 2 * dy - dx
            while x_start != x_end:
                x_start += x_inc
                if p >= 0:
                    y_start += y_inc
                    p -= 2 * dx
                p += 2 * dy
                if returnList:
                    traversed.append(np.array([x_start, y_start]))
                else:
                    if self.map[x_start][y_start] == self.occupied_space:
                        return True
        elif dy >= dx:
            p = 2 * dx - dy
            while y_start != y_end:
                y_start += y_inc
                if p >= 0:
                    x_start += x_inc
                    p -= 2 * dy
                p += 2 * dx
                if returnList:
                    traversed.append(np.array([x_start, y_start]))
                else:
                    if self.map[x_start][y_start] == self.occupied_space:
                        return True
        if returnList:
            return traversed
        else:
            return False

    def index_to_world(self, index):
        return self.bounds[0][0:2] + self.resolution*index + np.ones(2)*self.resolution/2


def plotTraj(A, bounds, walls):
    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection='3d')
    for obs, color in zip(walls, ['b', 'r', 'g', 'y']):
        verts = np.zeros((8, 3))
        i = 0
        for x in [obs["start"][0], obs["stop"][0]]:
            for y in [obs["start"][1], obs["stop"][1]]:
                for z in [obs["start"][2], obs["stop"][2]]:
                    verts[i] = np.array([x, y, z])
                    i += 1

        hull = ConvexHull(verts)
        for s in hull.simplices:
            sq = [[verts[s[0], 0], verts[s[0], 1], verts[s[0], 2]],
                  [verts[s[1], 0], verts[s[1], 1], verts[s[1], 2]],
                  [verts[s[2], 0], verts[s[2], 1], verts[s[2], 2]]]

            f = a3.art3d.Poly3DCollection([sq])
            f.set_color(color)
            f.set_edgecolor('k')
            f.set_alpha(1)
            ax.add_collection3d(f)

    q = A.goal
    while q is not A.root:
        ax.scatter(q.position[0], q.position[1], q.position[2])
        q = q.parent

    for i in ["x", "y", "z"]:
        eval("ax.set_{:s}label('{:s}')".format(i, i))

    ax.set_xlim(bounds[0][0], bounds[1][0])
    ax.set_ylim(bounds[0][1], bounds[1][1])
    ax.set_zlim(bounds[0][2], bounds[1][2])

    plt.show()


def main():
    """arg = 'home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json'

    with open(arg, 'rb') as f:
        world = json.load(f)"""

    airspace = {"min": [-10, -10, 0.0],
                "max": [ 10,  10, 5.0]}

    bounds = [np.array(airspace["min"]), np.array(airspace["max"])]

    walls = [{"start": [6.0, 3.0, 0.0], "stop": [2.0, 6.0, 3.0]},
             {"start": [-4.0, 4.0, 0.0], "stop": [-6.0, -0.5, 3.0]},
             {"start": [-10.0, -3.0, 0.0], "stop": [5.0, -5, 3.0]},
             {"start": [5.0, -7.5, 0.0], "stop": [6.0, -1.0, 3.0]}]

    radius = 0.2

    resolution = 0.4

    root_yaw = 0
    root_pos = np.array([-10, -8, 1])

    goal_yaw = 0
    goal_pos = np.array([-7.5, 2.5, 1])

    grid = GridMap(bounds, radius, resolution, walls, root_pos[2])

    root = Node(grid.convert_to_index(root_pos), None, root_pos, root_yaw)
    goal = Node(grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

    A = Planning(root, goal, grid)

    A.run()

    plotTraj(A, bounds, walls)


if __name__ == "__main__":
    main()
