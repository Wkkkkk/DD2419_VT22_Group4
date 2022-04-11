#!/usr/bin/env python

from math import *
import numpy as np
import rospy
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from grid_map import GridMap
from node import Node
from transform import Transform


class Planner:
    def __init__(self, start_pose, goal_pose, grid):
        self.tf = Transform()
        self.current_pose = None

        self.grid = grid

        #self.sub = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher('/mission_planner/path', Path, queue_size=10)

        self.start, self.goal = self.initialize_planning(start_pose, goal_pose)

        self.grid[self.start.index] = self.start
        self.grid[self.goal.index] = self.goal

        X = self.grid.dim[0]-1
        Y = self.grid.dim[1]-1
        self.neighbours = lambda x, y: [self.grid[(i, j)] for i in range(x-1, x+2) for j in range(y-1, y+2)
                                        if ((0 <= i <= X) and (0 <= j <= Y) and not (i == x and j == y)
                                            and self.grid[(i, j)] != self.grid.occupied_space)]

    def compute_cost(self, node):
        if node.parent is self.start:
            node.cost2come = np.linalg.norm(node.position - node.parent.position)
        else:
            node.cost2come = node.parent.cost2come + np.linalg.norm(node.position-node.parent.position)

        """H_path = self.grid.raytrace(node.index, self.goal.index, True)
        prev_cell = node.position
        for cell in H_path[1:]:
            pos2D = self.grid.index_to_world(cell)
            cell_pos = np.array([pos2D[0], pos2D[1], self.start.position[2]])
            node.cost2go += np.linalg.norm(prev_cell-cell_pos)
            prev_cell = cell_pos"""

        node.cost2go = np.linalg.norm(node.position-self.goal.position)
        node.cost = node.cost2go + node.cost2come

    def get_setpoints(self):
        tol = 10
        setpoints = []
        #setpoints.append(self.tf.pose_stamped_msg(self.goal.position, self.goal.yaw))
        goal = self.tf.pose_stamped_msg(self.goal.position, self.goal.yaw)
        node = self.goal
        while node is not self.start:
            q = node.parent
            while q is not None:
                if not self.grid.raytrace(node.index, q.index, False):
                    node.parent = q
                else:
                    break
                q = q.parent
            p = node.parent
            node.yaw = atan2(node.position[1] - p.position[1], node.position[0] - p.position[0])
            # yaw_diff = abs(np.degrees(p.yaw) - np.degrees(node.yaw))
            # if yaw_diff > tol and yaw_diff < 360-tol:
            #    setpoints.append(self.tf.pose_stamped_msg(node.position, p.yaw))
            setpoints.append(self.tf.pose_stamped_msg(node.position, node.yaw))
            p.position[2] = self.start.position[2]
            #setpoints.append(self.tf.pose_stamped_msg(p.position, p.yaw))
            node = p
        setpoints.reverse()
        return setpoints

    def run(self):
        rospy.loginfo("Path planner is running!")
        goal_found = False
        open_set = []
        closed_set = set()

        open_set.append(self.start)

        while len(open_set) > 0:
            node = open_set[0]
            for e in open_set[1:]:
                if e.cost <= node.cost:
                    if e.cost2go < node.cost2go:
                        node = e

            open_set.remove(node)
            closed_set.add(node)

            if node is self.goal:
                rospy.loginfo("Found the goal!")
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
            rospy.loginfo("Successfully found a trajectory!")
            setpoints = self.get_setpoints()
            self.path_publisher(setpoints)
            return setpoints
        else:
            rospy.loginfo("Could not find a trajectory!")
            return None

    def initialize_planning(self, start_pose,  goal_pose):

        start_yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        start_pos = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])

        goal_yaw = self.tf.quaternion2yaw(goal_pose.pose.orientation)
        goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])


        start = Node(self.grid.convert_to_index(start_pos), None, start_pos, start_yaw)

        goal = Node(self.grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

        return start, goal

    def path_publisher(self, setpoints):
        setpoints = [self.tf.pose_stamped_msg(self.start.position, self.start.yaw)] + setpoints
        path = Path()
        path.poses = setpoints
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        # print(path)
        self.pub.publish(path)
