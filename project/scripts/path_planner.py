#!/usr/bin/env python

from math import *
import numpy as np
import rospy
from nav_msgs.msg import Path
from node import Node
from transform import Transform
import random


class Planner:
    """ Class to generate an obstacle free path with A* algorithm """

    def __init__(self, grid):
        # Initialize class objects
        self.tf = Transform()
        self.grid = grid

        # Initialize path publisher
        self.pub = rospy.Publisher('/mission_planner/path', Path, queue_size=10)

        # Initialize variables
        self.current_pose = None
        self.loc_clusters = self.grid.loc_clusters
        self.start = None
        self.goal = None

        # Finds the neighbours defined by 8 connectivity of a grid cell
        self.neighbours = lambda x, y: [self.grid[(i, j)] for i in range(x - 1, x + 2) for j in range(y - 1, y + 2)
                                        if self.grid.is_in_bounds([i, j]) and not (i == x and j == y)
                                        and self.grid.is_free_space([i, j])]

    def compute_cost(self, node):
        """ Computes the cost of a node in a grid cell """
        if node.parent is self.start:
            node.cost2come = np.linalg.norm(node.position - node.parent.position)
        else:
            node.cost2come = node.parent.cost2come + np.linalg.norm(node.position-node.parent.position)

        node.cost2go = np.linalg.norm(node.position-self.goal.position)  # Heuristic cost
        node.cost = node.cost2go + node.cost2come  # Total cost

    def get_setpoints(self):
        """ Computes a minimum number of set points to follow the path, while considering poses by landmarks """

        setpoints = []
        node = self.goal

        loc_clusters_copy = self.loc_clusters[:]

        # If the goal is inside a localization cluster, remove the cluster from the cluster list.
        for cluster_index, cluster in enumerate(loc_clusters_copy):
            for loc_pose in cluster:
                if np.array_equal(loc_pose.index, node.index):
                    loc_clusters_copy.pop(cluster_index)

        # Iterate through all the nodes from goal to start
        while node is not self.start:
            q = node.parent
            while q is not None:
                # Check if there is an obstacle between the node and q
                if not self.grid.raytrace(node.index, q.index):
                    # If q is inside any localization cluster retrieve yaw and index of those clusters
                    cluster_indices = []
                    loc_yaw = []
                    for cluster_index, cluster in enumerate(loc_clusters_copy):
                        for loc_pose in cluster:
                            if np.array_equal(loc_pose.index, q.index):
                                loc_yaw.append(loc_pose.yaw)
                                cluster_indices.append(cluster_index)

                    if len(loc_yaw) > 0:
                        # Retrieve all poses included in the clusters which q coincides with
                        # and remove those clusters from the cluster list.
                        cluster_poses = []
                        for i in cluster_indices:
                            cluster_poses += loc_clusters_copy.pop(i)
                        cluster_nodes = []

                        # Retrieve all nodes in the path (that do not have an obstacle in the way)
                        # that are inside the clusters that q coincides with.
                        while q is not None:
                            for pose in cluster_poses:
                                if np.array_equal(pose.index, q.index):
                                    cluster_nodes.append(q)
                            if q.parent is not None and self.grid.raytrace(node.index, q.parent.index):
                                break
                            q = q.parent

                        if self.start in cluster_nodes:
                            q = self.start
                        else:
                            # Randomly select one of the nodes in these clusters.
                            q = random.choice(cluster_nodes)
                            q.yaw = random.choice(loc_yaw)

                        node.parent = q
                        break
                    else:
                        # If there isn't an obstacle in the way, the parent of the node is set to q.
                        node.parent = q
                else:
                    break
                q = q.parent

            p = node.parent

            # If the yaw of the node is not specified,
            # compute a yaw in direction from the parent node to the current node.
            if node.yaw is None:
                node.yaw = np.rad2deg(atan2(node.position[1] - p.position[1], node.position[0] - p.position[0]))

            p.position[2] = self.start.position[2]
            setpoints.append(self.tf.pose_stamped_msg(node.position, node.yaw))

            node = p

        setpoints.append(self.tf.pose_stamped_msg(self.start.position, self.start.yaw))
        setpoints.reverse()
        return setpoints

    def run(self, start_pose, goal_pose):
        """ Executing the A* algorithm """

        rospy.loginfo("Path planner is running!")

        self.initialize_planning(start_pose, goal_pose)

        goal_found = False
        open_set = []  # Priority queue
        closed_set = set()

        open_set.append(self.start)

        while len(open_set) > 0:
            node = open_set[0]
            for q in open_set[1:]:
                if q.cost <= node.cost:
                    if q.cost2go < node.cost2go:
                        node = q

            # Remove the node with the lowest cost from the priority queue
            open_set.remove(node)
            closed_set.add(node)

            if node is self.goal:
                rospy.loginfo("Found the goal!")
                goal_found = True
                break

            # Generate the neighbours of the lowest cost node
            neighbours = self.neighbours(node.index[0], node.index[1])
            for neighbour in neighbours:
                if neighbour in closed_set:
                    continue

                # Update the cost of the neighbours
                cost2neighbour = node.cost2come + np.linalg.norm(node.position-neighbour.position)
                if cost2neighbour < neighbour.cost2come or neighbour not in open_set:
                    neighbour.parent = node
                    self.compute_cost(neighbour)

                    if neighbour not in open_set:
                        open_set.append(neighbour)

        if goal_found:
            rospy.loginfo("Successfully found a trajectory!")
            setpoints = self.get_setpoints()
            self.path_publisher(setpoints)

            setpoints.pop(0)
            return setpoints
        else:
            rospy.loginfo("Could not find a trajectory!")
            return None

    def initialize_planning(self, start_pose,  goal_pose):
        """ Generates nodes for start pose and goal pose and inserts them in the grid map """

        start_yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        start_pos = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])

        goal_yaw = self.tf.quaternion2yaw(goal_pose.pose.orientation)
        goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        start = Node(self.grid.convert_to_index(start_pos), None, start_pos, start_yaw)

        goal = Node(self.grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

        self.start = start
        self.goal = goal

        self.grid[start.index] = start
        self.grid[goal.index] = goal

        return start, goal

    def path_publisher(self, setpoints):
        """ Publishes the path to be visualised in Rviz """
        setpoints = [self.tf.pose_stamped_msg(self.start.position, self.start.yaw)] + setpoints
        path = Path()
        path.poses = setpoints
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        self.pub.publish(path)

