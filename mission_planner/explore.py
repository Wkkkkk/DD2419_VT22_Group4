#!/usr/bin/env python

from grid_map import GridMap
from node import Node
import numpy as np
from transform import Transform as tr
from math import *


class Explore:

    def __init__(self, grid):
        self.grid = grid
        self.e_map = grid.reset_exlporation_map()
        self.range = 0.4

    def random_indices(self, current_index):
        N = 30
        rand_indices = np.zeros(N)
        for i in range(0, N):
            while True:
                x_rand = np.random.randint(self.grid.dim[0] + 1)
                y_rand = np.random.randint(self.grid.dim[1] + 1)
                rand_index = np.array([x_rand, y_rand])
                if self.grid(rand_index) != self.grid.occupied_space and \
                        np.linalg.norm(current_index-rand_index) < self.range:
                    break
            rand_indices[i] = (x_rand, y_rand)

        return rand_indices

    def exploration_score(self, current_index, rand_index):
        index_range = round(self.range/self.grid.resolution)
        x_max = rand_index[0] + index_range
        y_max = rand_index[1] + index_range
        x_min = rand_index[0] - index_range
        y_min = rand_index[1] - index_range

        border_indices = []
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                range_index = np.array([x, y])
                if index_range-1 < np.linalg.norm(rand_index - range_index) <= index_range:
                    border_indices.append(range_index)

        explored_cells = []
        cell_score_sum = 0
        for border_index in border_indices:
            traversed = self.grid.raytrace(rand_index, border_index, True)
            for index in traversed:
                explored_cells.append(index)
                cell_score = self.e_map[index[0]][index[1]]
                cell_score_sum += cell_score
                if cell_score == 3:
                    break

        dist = np.linalg.norm(current_index-rand_index)
        score = cell_score_sum/dist
        return score, explored_cells

    def next_point(self, current_pos):
        current_index = self.grid.convert_to_index(current_pos)
        found_point = False
        max_index = 0
        count = 0
        while not found_point and count < 20:
            rand_indices = self.random_indices(current_index)
            max_index = rand_indices[0]
            max_score, explored_cells = self.exploration_score(current_index, max_index)
            for rand_index in rand_indices[1:]:
                score, cells = self.exploration_score(current_index, rand_index)
                if score > max_score:
                    max_score = score
                    max_index = rand_index
                    explored_cells = cells
            if max_score != 0:
                found_point = True
            count += 1

        if found_point:
            for index in explored_cells:
                self.e_map[index[0]][index[1]] = 0
            yaw = np.random.uniform(-pi, pi)
            pos2D = self.grid.index_to_world(max_index)
            position = np.array([pos2D[0], pos2D[1], 0.4])
            next_pose = tr.pose_stamped(position, yaw)
            return next_pose
        else:
            return None

    def reset_map(self):
        self.e_map = self.grid.reset_exlporation_map()

