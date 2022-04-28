#!/usr/bin/env python

from math import *
import numpy as np
from node import Node
from tf.transformations import euler_matrix

class GridMap:
    def __init__(self, safety_radius, world, height):
        self.bounds = [np.array(world['airspace']["min"]), np.array(world['airspace']["max"])]
        self.height = height
        self.resolution = (self.bounds[1][0] - self.bounds[0][0])/30

        self.dim = ((self.bounds[1] - self.bounds[0]) / self.resolution).astype(int)

        self.occupied_space = 1
        self.c_space = 2
        self.safety_radius = safety_radius
        self.resolution = (self.bounds[1][0] - self.bounds[0][0]) / 20
        self.dim = ((self.bounds[1] - self.bounds[0]) / self.resolution).astype(int)

        self.map, self.explore_map = self.create_map(world['walls'])

        self.loc_clusters = self.get_localization_poses(world['markers'], world['roadsigns'])

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

        padding = int(round(self.safety_radius/self.resolution))

        for wall in walls:
            v1 = np.array(wall["plane"]["start"][0:2])
            v2 = np.array(wall["plane"]["stop"][0:2])

            index1 = self.convert_to_index(v1)
            index2 = self.convert_to_index(v2)
            occupied_cells = self.raytrace(index1, index2, True)

            for cell in occupied_cells:
                if self.is_in_bounds(cell):
                    map[cell[0]][cell[1]] = self.occupied_space
                    explore_map[cell[0]][cell[1]] = 0
                    x_min = cell[0] - padding
                    x_max = cell[0] + padding
                    y_min = cell[1] - padding
                    y_max = cell[1] + padding
                    for x in range(x_min, x_max + 1):
                        for y in range(y_min, y_max + 1):
                            if self.is_in_bounds([x, y]) and map[x][y] != self.occupied_space:
                                map[x][y] = self.c_space
                                explore_map[x][y] = 3

        for x in range(0, self.dim[0]):
            for y in range(0, self.dim[1]):
                if map[x][y] != self.occupied_space and map[x][y] != self.c_space:
                    index = np.array([x, y])
                    map[x][y] = Node(index, None, self.index_to_world(index, self.height))
        return map, explore_map

    def raytrace(self, start, end, returnList=False):
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

        traversed = [np.array([x_start, y_start])]
        if dx >= dy:
            p = 2 * dy - dx
            while x_start != x_end:
                x_start += x_inc
                if p >= 0:
                    y_start += y_inc
                    p -= 2 * dx
                p += 2 * dy
                if returnList:
                    if not self.is_in_bounds([x_start, y_start]):
                        break
                    traversed.append(np.array([x_start, y_start]))
                else:
                    if not self.is_free_space([x_start, y_start]) or not self.is_in_bounds([x_start, y_start]):
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
                    if not self.is_in_bounds([x_start, y_start]):
                        break
                    traversed.append(np.array([x_start, y_start]))
                else:
                    if not self.is_free_space([x_start, y_start]) or not self.is_in_bounds([x_start, y_start]):
                        return True
        if returnList:
            return traversed
        else:
            return False

    def landmark_poses(self, landmark):
        landmark_pos = landmark['pose']['position']
        orientation = landmark['pose']['orientation']
        landmark_index = self.convert_to_index(landmark_pos)
        self.force_in_bounds(landmark_index)

        M = euler_matrix(np.deg2rad(orientation[0]), np.deg2rad(orientation[1]), np.deg2rad(orientation[2]), axes='sxyz')
        M[:3, 3] = landmark_pos

        obs_pos_min = np.array([0, 0.3, -0.2, 1])
        obs_pos_max = np.array([0, 0.6, 0.2, 1])

        min_index = self.convert_to_index(np.dot(M, obs_pos_min))
        max_index = self.convert_to_index(np.dot(M, obs_pos_max))
        self.force_in_bounds(min_index)
        self.force_in_bounds(max_index)

        x_min = min_index[0]
        x_max = max_index[0]
        if x_min > x_max:
            swap = x_min
            x_min = x_max
            x_max = swap
        y_min = min_index[1]
        y_max = max_index[1]
        if y_min > y_max:
            swap = y_min
            y_min = y_max
            y_max = swap
        poses = []

        y_axis = np.array([0, 1, 0, 1])
        y_trans = np.dot(M, y_axis)
        direction = landmark_pos[0:2] - y_trans[0:2]
        yaw = round(np.rad2deg(atan2(direction[1], direction[0])))
        print("yaw: ", yaw)
        for x in range(x_min, x_max+1):
            for y in range(y_min, y_max+1):
                if self.is_free_space([x, y]):
                    index = np.array([x, y])
                    pos = self.index_to_world(index, self.height)
                    node = Node(index, None, pos, yaw)
                    poses.append(node)

        return poses

    def force_in_bounds(self, index):
        if index[0] > self.dim[0] - 1:
            index[0] = self.dim[0] - 1
        if index[1] > self.dim[1] - 1:
            index[1] = self.dim[1] - 1
        if index[0] < 0:
            index[0] = 0
        if index[1] < 0:
            index[1] = 0

    def get_localization_poses(self, markers, signs):
        loc_clusters = []
        for marker in markers:
            poses = self.landmark_poses(marker)
            loc_clusters.append(poses)
        for sign in signs:
            poses = self.landmark_poses(sign)
            loc_clusters.append(poses)
        return loc_clusters

    def is_in_bounds(self, index):
        if index[0] < 0 or index[1] < 0 or index[0] >= self.dim[0] or index[1] >= self.dim[1]:
            return False
        else:
            return True

    def is_free_space(self, index):
        if self.map[index[0]][index[1]] == self.occupied_space or self.map[index[0]][index[1]] == self.c_space:
            return False
        else:
            return True

    def index_to_world(self, index, z):
        pos2D = self.bounds[0][0:2] + self.resolution*index + np.ones(2)*self.resolution/2
        pos3D = np.append(pos2D, z)
        return pos3D

    def reset_exploration_map(self):
        return self.explore_map.copy()

