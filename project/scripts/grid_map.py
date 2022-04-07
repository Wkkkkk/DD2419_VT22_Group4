#!/usr/bin/env python

from math import *
import numpy as np
from node import Node


class GridMap:
    def __init__(self, radius, world):
        self.bounds = [np.array(world['airspace']["min"]), np.array(world['airspace']["max"])]
        self.occupied_space = 1
        self.radius = radius
        self.resolution = (self.bounds[1][0] - self.bounds[0][0])/20
        print(self.resolution)

        self.dim = ((self.bounds[1] - self.bounds[0]) / self.resolution).astype(int)

        self.map, self.explore_map = self.create_map(world['walls'])

    def __getitem__(self, index):
        x, y = index
        return self.map[x][y]

    def __setitem__(self, index, val):
        x, y = index
        self.map[x][y] = val

    def convert_to_index(self, pos):
        float_index = (pos[0:2] - self.bounds[0][0:2])/self.resolution
        return float_index.astype(int)

    def create_map(self, walls):
        map = np.empty((self.dim[0], self.dim[1]), dtype=object)
        explore_map = np.ones((self.dim[0], self.dim[1]), dtype=object)

        padding = int(round(self.radius/self.resolution))

        for wall in walls:
            v1 = np.array(wall["plane"]["start"][0:-1])
            v2 = np.array(wall["plane"]["stop"][0:-1])

            index1 = self.convert_to_index(v1)
            index2 = self.convert_to_index(v2)
            occupied_cells = self.raytrace(index1, index2, True)

            for cell in occupied_cells:
                x_min = cell[0]-padding
                x_max = cell[0]+padding
                y_min = cell[1]-padding
                y_max = cell[1]+padding
                for x in range(x_min, x_max+1):
                    for y in range(y_min, y_max+1):
                        if self.dim[0]-1 >= x >= 0 and self.dim[1]-1 >= y >= 0:
                            map[x][y] = self.occupied_space
                            if (x == x_min or x == x_max or y == y_min or y == y_max) and explore_map[x][y] != 0:
                                explore_map[x][y] = 3
                            else:
                                explore_map[x][y] = 0

        for x in range(0, self.dim[0]):
            for y in range(0, self.dim[1]):
                if map[x][y] != self.occupied_space:
                    index = np.array([x, y])
                    pos2D = self.index_to_world(index)
                    map[x][y] = Node(index, None, np.array([pos2D[0], pos2D[1], 0.0]))
        return map, explore_map

    def occupied_cell(self, index):
        if self.map[index[0]][index[1]] == self.occupied_space:
            return True
        else:
            return False

    def raytrace(self, start, end, returnList):
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
                    if x_start < 0 or y_start < 0 or x_start > self.dim[0]-1 or y_start > self.dim[1]-1:
                        break
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
                    if x_start < 0 or y_start < 0 or x_start > self.dim[0]-1 or y_start > self.dim[1]-1:
                        break
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

    def reset_exploration_map(self):
        return self.explore_map.copy()

