#!/usr/bin/env python

from grid_map import GridMap
from node import Node
import numpy as np
from transform import Transform
import random
import itertools


class Explore:

    def __init__(self, grid, current_pose):
        self.grid = grid
        self.loc_clusters = self.grid.loc_clusters
        self.e_map = grid.reset_exploration_map()
        self.range = int(0.5/self.grid.resolution)

        self.exploration_mode = 10
        self.localization_mode = 20
        self.mode = self.localization_mode

        current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
        self.current_index = self.grid.convert_to_index(current_position)
        score, explored_cells = self.exploration(self.current_index)
        for index in explored_cells:
            self.e_map[index[0]][index[1]] = 0
        self.tf = Transform()


    def get_random_poses(self):
        N = 30
        rand_poses = np.empty(N, dtype=object)
        np.random.seed(2020)
        for i in range(0, N):
            while True:
                x_rand = np.random.randint(self.grid.dim[0])
                y_rand = np.random.randint(self.grid.dim[1])
                rand_index = np.array([x_rand, y_rand])
                if self.grid.is_free_space(rand_index) and \
                        np.linalg.norm(self.current_index-rand_index) > 2*self.range:
                    break
            rand_pose = self.grid[rand_index]
            loc_yaw = []
            for cluster in self.loc_clusters:
                for loc_pose in cluster:
                    if rand_pose.index[0] == loc_pose.index[0] and rand_pose.index[1] == loc_pose.index[1]:
                        loc_yaw.append(loc_pose.yaw)
            if len(loc_yaw) > 0:
                rand_pose.yaw = random.choice(loc_yaw)
            else:
                rand_pose.yaw = np.random.uniform(-180, 180)
            rand_poses[i] = rand_pose

        return rand_poses

    def get_localization_pose(self):
        loc_poses = list(itertools.chain.from_iterable(self.loc_clusters))
        filtered_poses = list(filter(lambda pose : np.linalg.norm(pose.index-self.current_index) > 2*self.range, loc_poses))
        sorted_poses = sorted(filtered_poses, key=lambda pose: np.linalg.norm(pose.index-self.current_index))

        next_pose = None

        for i, pose in enumerate(sorted_poses):
            if not self.grid.raytrace(pose.index, self.current_index):
                same_spot = []
                same_spot.append(pose)
                k = 1
                while i < len(sorted_poses)-1 and np.array_equal(pose.index, sorted_poses[i+k].index):
                    same_spot.append(sorted_poses[i+k])
                    k += 1
                if len(same_spot) > 1:
                    next_pose = random.choice(same_spot)
                else:
                    next_pose = pose

        if next_pose is None:
            next_pose = sorted_poses[0]

        return next_pose

    def exploration(self, center_index):
        self.grid.force_in_bounds(center_index)

        x_max = center_index[0] + self.range
        y_max = center_index[1] + self.range
        x_min = center_index[0] - self.range
        y_min = center_index[1] - self.range

        free_cells = []
        c_cells = []
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                circle_index = np.array([x, y])
                if self.grid.is_in_bounds([x, y]) and np.linalg.norm(center_index - circle_index) <= self.range:
                    if self.e_map[x][y] > 1:
                        c_cells.append(circle_index)
                    elif self.e_map[x][y] == 1:
                        free_cells.append(circle_index)

        explored_cells = []
        explored_cells.append(center_index)

        score = 0
        for index in free_cells:
            if not self.grid.raytrace(center_index, index):
                score += self.e_map[index[0]][index[1]]
                explored_cells.append(index)

        for index in c_cells:
            traversed = self.grid.raytrace(center_index, index, True)
            for cell in traversed:
                if self.grid[cell] == self.grid.c_space:
                    if all(index == cell):
                        score += self.e_map[index[0]][index[1]]
                        explored_cells.append(index)
                    break

        dist = np.linalg.norm(self.current_index - center_index)
        dist_weight = 10 / self.grid.resolution
        if dist != 0 and score > 0:
            score = score + dist_weight / dist
        return score, explored_cells

    def next_goal(self, current_pose):
        current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
        self.current_index = self.grid.convert_to_index(current_position)

        valid_goal = False
        max_pose = None
        max_score = 0
        count = 0

        if self.mode == self.exploration_mode:
            while not valid_goal and count < 5:
                rand_poses = self.get_random_poses()
                max_pose = rand_poses[0]
                max_score, explored_cells = self.exploration(max_pose.index)
                for rand_pose in rand_poses[1:]:
                    score, cells = self.exploration(rand_pose.index)
                    if score > max_score:
                        max_score = score
                        max_pose = rand_pose
                        explored_cells = cells
                if max_score != 0:

                    valid_goal = True
                count += 1

            self.mode = self.localization_mode
            print("********** Random mode **********")
            if valid_goal and max_score > 1 / self.grid.resolution:
                for index in explored_cells:
                    self.e_map[index[0]][index[1]] = 0
                next_pose = self.tf.pose_stamped_msg(max_pose.position, max_pose.yaw)
                return next_pose
            else:
                return None

        else:
            self.mode = self.exploration_mode
            localization_pose = self.get_localization_pose()
            _, explored_cells = self.exploration(localization_pose.index)
            for index in explored_cells:
                self.e_map[index[0]][index[1]] = 0
            print("********** Localization mode **********")
            next_pose = self.tf.pose_stamped_msg(localization_pose.position, localization_pose.yaw)
            print("localization yaw (exploration): ", localization_pose.yaw)
            return next_pose

    def reset_map(self):
        self.e_map = self.grid.reset_exlporation_map()

