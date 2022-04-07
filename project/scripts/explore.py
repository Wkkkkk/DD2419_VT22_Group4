#!/usr/bin/env python

from grid_map import GridMap
from node import Node
import numpy as np
from transform import Transform
from math import *


class Explore:

    def __init__(self, grid, current_pose):
        self.grid = grid
        self.e_map = grid.reset_exploration_map()
        self.range = int(0.5/self.grid.resolution)

        current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
        self.current_index = self.grid.convert_to_index(current_position)
        score, explored_cells = self.exploration(self.current_index)
        for index in explored_cells:
            self.e_map[index[0]][index[1]] = 0
        self.tf = Transform()

    def random_indices(self):
        N = 10
        rand_indices = np.empty(N, dtype=object)
        for i in range(0, N):
            while True:
                x_rand = np.random.randint(self.grid.dim[0])
                y_rand = np.random.randint(self.grid.dim[1])
                rand_index = np.array([x_rand, y_rand])
                if self.grid[rand_index] != self.grid.occupied_space and \
                        np.linalg.norm(self.current_index-rand_index) > 2*self.range:
                    break
            rand_indices[i] = np.array([x_rand, y_rand])

        return rand_indices

    def exploration(self, goal_index):
        x_max = goal_index[0] + self.range
        y_max = goal_index[1] + self.range
        x_min = goal_index[0] - self.range
        y_min = goal_index[1] - self.range

        border_indices = []
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                range_index = np.array([x, y])
                if self.range-1 < np.linalg.norm(goal_index - range_index) <= self.range:
                    border_indices.append(range_index)

        if goal_index[0] > self.grid.dim[0]-1:
            goal_index[0] = self.grid.dim[0]-1
        if goal_index[1] > self.grid.dim[1]-1:
            goal_index[1] = self.grid.dim[1]-1

        explored_cells = []
        explored_cells.append(goal_index)

        score = 0
        for border_index in border_indices:
            traversed = self.grid.raytrace(goal_index, border_index, True)
            for index in traversed:
                explored_cells.append(index)
                cell_score = self.e_map[index[0]][index[1]]
                score += cell_score
                if self.grid[index] == self.grid.occupied_space:
                    break

        dist = np.linalg.norm(self.current_index-goal_index)
        dist_weight = 5/self.grid.resolution
        if dist != 0 and score > 0:
            score = score + dist_weight/dist

        return score, explored_cells

    def next_goal(self, current_pose):
        current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
        self.current_index = self.grid.convert_to_index(current_position)

        found_point = False
        max_index = 0
        max_score = 0
        count = 0
        while not found_point and count < 5:
            rand_indices = self.random_indices()
            max_index = rand_indices[0]
            max_score, explored_cells = self.exploration(max_index)
            for rand_index in rand_indices[1:]:
                score, cells = self.exploration(rand_index)
                if score > max_score:
                    max_score = score
                    max_index = rand_index
                    explored_cells = cells
            if max_score != 0:
                found_point = True
            count += 1

        if found_point and max_score > 2.5/self.grid.resolution:
            for index in explored_cells:
                self.e_map[index[0]][index[1]] = 0
            yaw = np.random.uniform(-pi, pi)
            pos2D = self.grid.index_to_world(max_index)
            position = np.array([pos2D[0], pos2D[1], 0.4])
            next_pose = self.tf.pose_stamped_msg(position, yaw)
            return next_pose
        else:
            return None

    def reset_map(self):
        self.e_map = self.grid.reset_exlporation_map()

