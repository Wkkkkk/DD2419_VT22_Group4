#!/usr/bin/env python

import numpy as np
from transform import Transform
import random
import itertools


class Explore:
    """ Class used for exploring the world """

    def __init__(self, grid, start_pose):
        self.grid = grid
        self.loc_clusters = self.grid.loc_clusters
        self.e_map = grid.reset_exploration_map()

        self.tf = Transform()

        # Range of which the drone can observe translated to grid map index
        self.range = int(0.5/self.grid.resolution)

        # Modes of exploration
        self.random_mode = 10
        self.localization_mode = 20

        # Starting position of the drone
        start_position = self.tf.posestamped_to_array(start_pose)

        # Initialize exploration
        self.mode = self.localization_mode
        self.current_index = self.grid.convert_to_index(start_position)
        score, explored_cells = self.exploration(self.current_index)
        for index in explored_cells:
            self.e_map[index[0]][index[1]] = 0

    def get_random_poses(self):
        """ Generate N random poses """

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

            # Set the yaw of the random pose to a yaw that observes a landmark
            # if the random position coincides with a "localization position"
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
        """ Choose one of the "localization poses" to make the drone fly in front of a landmark """

        loc_poses = list(itertools.chain.from_iterable(self.loc_clusters))

        # Only keep the poses that are 2*range away from the current position
        filtered_poses = list(filter(lambda pose : np.linalg.norm(pose.index-self.current_index) > 2*self.range, loc_poses))

        # Sort the poses based on distance from the current position
        sorted_poses = sorted(filtered_poses, key=lambda pose: np.linalg.norm(pose.index-self.current_index))

        # Choose the nearest pose that is not occluded by an obstacle
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
        """ Calculate exploration score and return the grid cells that have been explored """

        self.grid.force_in_bounds(center_index)

        x_max = center_index[0] + self.range
        y_max = center_index[1] + self.range
        x_min = center_index[0] - self.range
        y_min = center_index[1] - self.range

        # Divide all the cells to free cells and c cells (padding),
        # within a radius equal to self.range from the current index
        free_cells = []
        c_cells = []
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                circle_index = np.array([x, y])
                if self.grid.index_in_bounds([x, y]) and np.linalg.norm(center_index - circle_index) <= self.range:
                    if self.e_map[x][y] > 1:
                        c_cells.append(circle_index)
                    elif self.e_map[x][y] == 1:
                        free_cells.append(circle_index)

        explored_cells = []
        explored_cells.append(center_index)

        # Sum the score for all free cells in the circle
        score = 0
        for index in free_cells:
            if not self.grid.raytrace(center_index, index):
                score += self.e_map[index[0]][index[1]]
                explored_cells.append(index)

        # Sum the score of all c cells in the circle that have no obstacle in the way
        for index in c_cells:
            traversed = self.grid.raytrace(center_index, index, True)
            for cell in traversed:
                if self.grid[cell] == self.grid.c_space:
                    if all(index == cell):
                        score += self.e_map[index[0]][index[1]]
                        explored_cells.append(index)
                    break

        # Add another score inversely proportional to the distance from the current position
        dist = np.linalg.norm(self.current_index - center_index)
        dist_weight = 10 / self.grid.resolution
        if dist != 0 and score > 0:
            score = score + dist_weight / dist
        return score, explored_cells

    def next_goal(self, current_pose):
        """ Generates the next exploration goal based on the current pose """

        current_position = self.tf.posestamped_to_array(current_pose)
        self.current_index = self.grid.convert_to_index(current_position)

        valid_goal = False
        best_pose = None
        best_score = 0
        count = 0

        # enerate random poses if in random mode and select the pose with the highest score
        if self.mode == self.random_mode:
            print("********** Random mode **********")

            self.mode = self.localization_mode

            while not valid_goal and count < 5:
                rand_poses = self.get_random_poses()
                best_pose = rand_poses[0]
                best_score, explored_cells = self.exploration(best_pose.index)
                for rand_pose in rand_poses[1:]:
                    score, cells = self.exploration(rand_pose.index)
                    if score > best_score:
                        best_score = score
                        best_pose = rand_pose
                        explored_cells = cells
                if best_score != 0:
                    valid_goal = True
                count += 1

            # End exploration if score is below a threshold
            if valid_goal and best_score > 1 / self.grid.resolution:

                for index in explored_cells:
                    self.e_map[index[0]][index[1]] = 0

                next_pose = self.tf.pose_stamped_msg(best_pose.position, best_pose.yaw)
                return next_pose
            else:
                return None

        else:
            # Select a localization pose if in localization mode

            print("********** Localization mode **********")

            self.mode = self.random_mode

            localization_pose = self.get_localization_pose()
            _, explored_cells = self.exploration(localization_pose.index)

            for index in explored_cells:
                self.e_map[index[0]][index[1]] = 0

            next_pose = self.tf.pose_stamped_msg(localization_pose.position, localization_pose.yaw)
            return next_pose

    def reset_map(self):
        self.e_map = self.grid.reset_exlporation_map()