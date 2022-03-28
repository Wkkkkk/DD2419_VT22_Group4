#!/usr/bin/env python

import json
from math import *
import numpy as np
import rospy
import math
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def pose_callback(msg):
    global cf_pose
    cf_pose = msg


def goal_callback(msg):
    global goal_pose
    goal_pose = msg


class Node:
    def __init__(self, index, parent=None, position=np.zeros(3), yaw=0.0):
        self.index = index
        self.position = position
        self.yaw = yaw
        self.parent = parent
        self.children = []
        self.cost2go = 0
        self.cost2come = 0
        self.cost = 0


class Planning:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        grid[start.index] = start
        grid[goal.index] = goal

        self.grid = grid

        X = grid.dim[0]
        Y = grid.dim[1]
        self.neighbours = lambda x, y: [grid[(i, j)] for i in range(x-1, x+2)
                                         for j in range(y-1, y+2)
                                         if ((0 <= i <= X) and (0 <= j <= Y) and i != x and j != y
                                             and grid[(i, j)] != grid.occupied_space)]

    def compute_cost(self, node):
        if node.parent is self.start:
            node.cost2come = np.linalg.norm(node.position - node.parent.position)
        else:
            node.cost2come = node.parent.cost2come + np.linalg.norm(node.position-node.parent.position)

        H_path = self.grid.trajectory(node.index, self.goal.index, True)

        prev_cell = node.position
        for cell in H_path[1:]:
            pos2D = self.grid.index_to_world(cell)
            cell_pos = np.array([pos2D[0], pos2D[1], self.start.position[2]])
            node.cost2go += np.linalg.norm(prev_cell-cell_pos)
            prev_cell = cell_pos

        node.cost = node.cost2go + node.cost2come

    def run(self):
        rospy.loginfo("******************* Path planner is running *******************");
        goal_found = False
        open_set = []
        closed_set = set()

        open_set.append(self.start)

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
            setpoints = []
            goal = pose_stamped(self.goal)
            setpoints.append(goal)
            node = self.goal
            while node.parent is not None:
                q = node.parent
                while q.parent is not None:
                    q = q.parent
                    if not self.grid.trajectory(node.index, q.index, False):
                        node.parent = q
                    else:
                        break
                p = node.parent
                p.yaw = atan2(node.position[1] - p.position[1], node.position[0] - p.position[0])
                setpoint = pose_stamped(p)
                setpoints.append(setpoint)
                node = p
            return setpoints.reverse()

        else:
            return None


class GridMap:
    def __init__(self, bounds, radius, resolution, obstacles):
        self.occupied_space = 1
        self.radius = radius
        self.bounds = bounds
        self.resolution = resolution

        X = int((self.bounds[1][0] - self.bounds[0][0]) / self.resolution) + 1
        Y = int((self.bounds[1][1] - self.bounds[0][1]) / self.resolution) + 1
        self.dim = [X - 1, Y - 1]
        self.map = self.create_map(obstacles)

    def __getitem__(self, index):
        x, y = index
        return self.map[x][y]

    def __setitem__(self, index, val):
        x, y = index
        self.map[x][y] = val

    def convert_to_index(self, pos):
        float_index = (pos[0:2] - self.bounds[0][0:2])/self.resolution
        return float_index.astype(int)

    def create_map(self, obstacles):
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
                    map[x][y] = Node(index, None, np.array([pos2D[0], pos2D[1], start_pos[2]]))

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


def get_yaw(q):
    return atan2(2 * (q.w * q.z + q.x * q.y),
                 1 - 2 * (q.y * q.y + q.z * q.z))


def yaw2quaternion(yaw):
    return euler_from_quaternion((0, 0, yaw))


def quaternion2yaw(orientation):
    roll, pitch, yaw = euler_from_quaternion((orientation.x,
                                              orientation.y,
                                              orientation.z,
                                              orientation.w))
    return yaw


def pose_stamped(node):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = node.posiition[0]
    msg.pose.position.y = node.posiition[1]
    msg.pose.position.z = node.posiition[2]
    q = yaw2quaternion(node.yaw)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    msg.header.frame_id = 'map'
    return msg


def path_publisher(setpoints):
    path = Path()
    path.poses = setpoints
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()
    pub.publish(path)



def main():
    global start_pos
    arg = 'home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json'

    with open(arg, 'rb') as f:
        world = json.load(f)

    airspace = world['airspace']

    walls = world['walls']

    bounds = [np.array(airspace["min"]), np.array(airspace["max"])]

    while not goal_pose:
        continue

    while not cf_pose:
        continue

    root_pose = cf_pose

    root_yaw = get_yaw(cf_pose.pose.orientation)
    root_pos = np.array(root_pose.pose.position)
    start_pos = root_pos

    goal_yaw = get_yaw(goal_pose.pose.orientation)
    goal_pos = np.array(goal_pose.pose.position)

    if root_pos[2] == 0:
        start_pos[2] = goal_pos[2]

    radius = 0.2

    resolution = 0.4

    grid = GridMap(bounds, radius, resolution, walls)

    root = Node(grid.convert_to_index(root_pos), None, root_pos, root_yaw)
    if root_pos[2] == 0:
        start = Node(grid.convert_to_index(start_pos), None, start_pos, root_yaw)
    else:
        start = root
    goal = Node(grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

    A = Planning(start, goal, grid)

    setpoints = A.run()

    if setpoints:
        rospy.loginfo("Successfully found a trajectory!")
        path_publisher(setpoints)
    else:
        rospy.loginfo("Could not find a trajectory!")


if __name__ == "__main__":
    rospy.init_node('path_planning')

    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/new_goal', PoseStamped, goal_callback)

    pub = rospy.Publisher('/trajectory', Path, queue_size=2)

    cf_pose = None

    goal_pose = None

    main()

    rospy.spin()