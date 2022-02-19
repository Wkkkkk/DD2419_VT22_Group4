 import json
from math import *
import numpy as np
import matplotlib.pyplot as plt     # Comment out if you are going to simulate in Matlab
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import mpl_toolkits.mplot3d as a3


class Node:
    def __init__(self, index, parent=None, yaw=0):
        self.x = index[0]
        self.y = index[1]
        self.z = index[2]
        self.yaw = yaw
        self.parent = parent
        self.children = []
        self.cost2go = 0
        self.cost2come = 0
        self.cost = 0


class Planning:
    def __init__(self, root, goal, grid):
        self.real_goal = (goal.x, goal.y, goal.z)
        self.root = root
        root.x, root.y, root.z = grid.convert_to_index((root.x,root.y,root.z))
        grid[(root.x, root.y, root.z)] = grid.visited_space
        self.goal = goal
        goal.x, goal.y, goal.z = grid.convert_to_index((goal.x, goal.y, goal.z))
        self.grid = grid
        X = grid.dim[0]
        Y = grid.dim[1]
        Z = grid.dim[2]
        self.neighbors = lambda x, y, z: [(i, j, k) for i in range(x - 1, x + 2)
                                         for j in range(y - 1, y + 2)
                                         for k in range(z - 1, z + 2)
                                         if ((0 <= i <= X) and (0 <= j <= Y) and (0 <= k <= Z)
                                             and grid[(i, j, k)] == grid.free_space)]

    def compute_cost(self, node):
        pos_child = np.array([node.x, node.y, node.z])
        pos_parent = np.array([node.parent.x, node.parent.y, node.parent.z])
        pos_goal = np.array([self.goal.x, self.goal.y, self.goal.z])
        node.cost2come = node.parent.cost2come + np.linalg.norm(pos_child-pos_parent)
        node.cost2go = np.linalg.norm(pos_child-pos_goal)
        node.cost = node.cost2go + node.cost2come

    def run(self):
        p = self.root
        goal_found = False
        while not goal_found:
            children = self.neighbors(p.x, p.y, p.z)
            best_node = Node(children[0], p)
            self.grid[(best_node.x, best_node.y, best_node.z)] = self.grid.visited_space
            self.compute_cost(best_node)
            for child in children[1:]:
                node = Node(child, p)
                self.grid[(node.x, node.y, node.z)] = self.grid.visited_space
                self.compute_cost(node)
                if node.x == self.goal.x and node.y == self.goal.y and node.z == self.goal.z:
                    goal_found = True
                    self.goal.parent = p
                    break

                if node.cost < best_node.cost:
                    best_node = node
            p = best_node

        self.goal.x, self.goal.y, self.goal.z = self.real_goal
        q = self.goal.parent
        while q is not self.root:
            q.x, q.y, q.z = self.grid.index_to_world(np.array([q.x, q.y, q.z]))
            q = q.parent


class GridMap:
    def __init__(self, bounds, radius, resolution):
        self.occupied_space = 1
        self.free_space = 0
        self.visited_space = 2
        self.radius = radius
        self.bounds = bounds
        self.resolution = resolution
        x = int((bounds[1][0] - bounds[0][0]) / resolution)+1
        y = int((bounds[1][1] - bounds[0][1]) / resolution)+1
        z = int((bounds[1][2] - bounds[0][2]) / resolution)+1
        self.map = np.zeros((x, y, z))
        self.dim = [x-1, y-1, z-1]

    def __getitem__(self, pos):
        x, y, z = pos
        return self.map[x][y][z]

    def __setitem__(self, pos, val):
        x, y, z = pos
        self.map[x][y][z] = val

    def convert_to_index(self, pos):
        float_index = (pos - self.bounds[0])/self.resolution
        return float_index.astype(int)

    def insert_obstacles(self, obstacles):
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
            if v1[2] > v2[2]:
                sign2[2] = -1
            else:
                sign1[2] = -1

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
            if index1[2] > index2[2]:
                min_z = index2[2]
                max_z = index1[2]
            else:
                min_z = index1[2]
                max_z = index2[2]

            for x in range(min_x, max_x+1):
                for y in range(min_y, max_y+1):
                    for z in range(min_z, max_z+1):
                        self.map[x][y][z] = self.occupied_space

    def occupied_cell(self, index):
        if self.map[index[0]][index[1]][index[2]] == self.occupied_space:
            return True
        else:
            return False

    """def occupied_trajectory(self, start, end):
        (start_x, start_y, start_z) = start
        (end_x, end_y, end_z) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return False"""

    def index_to_world(self, index):
        return self.bounds[0] + self.resolution*index + np.ones(3)*self.resolution/2


def plotTraj(A, bounds, walls):
    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection='3d')
    n = 4
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
        ax.scatter(q.x, q.y, q.z)
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
             {"start": [-4.0, 4.0, 0.0], "stop": [-6.0, 1.0, 3.0]},
             {"start": [-10.0, -3.0, 0.0], "stop": [-5.0, -4.0, 3.0]},
             {"start": [5.0, -4.0, 0.0], "stop": [6.0, -1.0, 3.0]}]

    radius = 0.5

    resolution = 0.5

    grid = GridMap(bounds, radius, resolution)

    grid.insert_obstacles(walls)

    root = Node([-10, -8, 1])
    goal = Node([10, 10, 3])

    A = Planning(root, goal, grid)

    A.run()

    plotTraj(A, bounds, walls)


if __name__ == "__main__":
    main()
