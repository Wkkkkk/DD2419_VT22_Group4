#!/usr/bin/env python

import json
from math import *
import numpy as np
import sys
import rospy
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def pose_callback(msg):
    global cf_pose

    cf_pose = transform2map(msg)


def goal_callback(msg):
    global goal_pose
    goal_pose = msg


def transform2map(m):
    timeout = rospy.Duration(0.5)
    if not tf_buffer.can_transform(m.header.frame_id, 'map', m.header.stamp, timeout):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % m.header.frame_id)
        return

    goal_map = tf_buffer.transform(m, 'map')
    goal_map.header.frame_id = 'map'
    goal_map.header.stamp = m.header.stamp

    return goal_map
    # goal = PoseStamped()
    # goal.header.stamp = m.header.stamp
    # goal.x = goal_map.pose.position.x
    # goal.y = goal_map.pose.position.y
    # goal.z = goal_map.pose.position.z
    # goal.header.frame_id = 'map'
    # roll, pitch, yaw = euler_from_quaternion((goal_map.pose.orientation.x,
    #                                           goal_map.pose.orientation.y,
    #                                           goal_map.pose.orientation.z,
    #                                           goal_map.pose.orientation.w))
    # goal.yaw = degrees(yaw)
    # return goal


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

        #H_path = self.grid.raytrace(node.index, self.goal.index, True)

        """prev_cell = node.position
        for cell in H_path[1:]:
            pos2D = self.grid.index_to_world(cell)
            cell_pos = np.array([pos2D[0], pos2D[1], self.start.position[2]])
            node.cost2go += np.linalg.norm(prev_cell-cell_pos)
            prev_cell = cell_pos"""

	node.cost2go = np.linalg.norm(node.position-self.goal.position)

        node.cost = node.cost2go + node.cost2come

    def run(self):
        rospy.loginfo("Path planner is running!");
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
            while node is not self.start:
                q = node.parent
                while q.parent is not None:
                    if not self.grid.raytrace(node.index, q.index, False):
                        node.parent = q
                    else:
                        break
                    q = q.parent
                p = node.parent
                p.yaw = atan2(node.position[1] - p.position[1], node.position[0] - p.position[0])
                setpoint = pose_stamped(p)
                setpoints.append(setpoint)
                node = p
            setpoints.reverse()
            return setpoints
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

        cells_to_add = int(round(self.radius/self.resolution))

        for obs in obstacles:
            v1 = np.array(obs["plane"]["start"][0:-1])
            v2 = np.array(obs["plane"]["stop"][0:-1])

            sign1 = np.ones(2)
            sign2 = np.ones(2)

            if v1[0] > v2[0]:
                sign2[0] = -1
            else:
                sign1[0] = -1
            if v1[1] > v2[1]:
                sign2[1] = -1
            else:
                sign1[1] = -1

            if v1[0]+self.radius > self.bounds[1][0] or v1[0]-self.radius < self.bounds[0][0]:
                sign1[0] = 0
            if v1[1]+self.radius > self.bounds[1][1] or v1[1]-self.radius < self.bounds[0][1]:
                sign1[1] = 0
            if v2[0]+self.radius > self.bounds[1][0] or v2[0]-self.radius < self.bounds[0][0]:
                sign2[0] = 0
            if v2[1]+self.radius > self.bounds[1][1] or v2[1]-self.radius < self.bounds[0][1]:
                sign2[1] = 0

            index1 = self.convert_to_index(v1+self.radius*sign1)
            index2 = self.convert_to_index(v2+self.radius*sign2)
            occupied_cells = self.raytrace(index1, index2, True)

            for cell in occupied_cells:
                for x in range(cell[0]-cells_to_add,cell[0]+cells_to_add+1):
                    for y in range(cell[1]-cells_to_add,cell[1]+cells_to_add+1):
                        if self.dim[0] >= x >= 0 and self.dim[1] >= y >= 0:
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
    return quaternion_from_euler(0.0, 0.0, yaw)


def pose_stamped(node):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = node.position[0]
    msg.pose.position.y = node.position[1]
    msg.pose.position.z = node.position[2]
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


def initialize_planning(world):
    global start_pos, goal_pose, cf_pose
    cf_pose = None
    goal_pose = None

    airspace = world['airspace']
    walls = world['walls']
    bounds = [np.array(airspace["min"]), np.array(airspace["max"])]

    while not goal_pose:
        continue

    while not cf_pose:
        continue

    root_pose = cf_pose

    root_yaw = get_yaw(cf_pose.pose.orientation)
    root_pos = np.array([root_pose.pose.position.x, root_pose.pose.position.y, root_pose.pose.position.z])
    start_pos = root_pos.copy()

    goal_yaw = get_yaw(goal_pose.pose.orientation)
    goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])
    tol = 1e-1

    if root_pos[2] < tol:
        start_pos[2] = goal_pos[2]

    radius = 0.1

    resolution = (bounds[1][0] - bounds[0][0])/20

    grid = GridMap(bounds, radius, resolution, walls)

    rospy.loginfo("Grid map created!")

    root = Node(grid.convert_to_index(root_pos), None, root_pos, root_yaw)

    if root_pos[2] < tol:
        start = Node(grid.convert_to_index(start_pos), root, start_pos, root_yaw)
    else:
        start = root
    goal = Node(grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

    A = Planning(start, goal, grid)

    setpoints = A.run()

    if setpoints is not None:
        rospy.loginfo("Successfully found a trajectory!")
        path_publisher(setpoints)
    else:
        rospy.loginfo("Could not find a trajectory!")


def main(argv=sys.argv):

    args = rospy.myargv(argv=argv)
    with open(args[1], 'rb') as f:
        world = json.load(f)

    #while not rospy.is_shutdown():
    initialize_planning(world)


if __name__ == "__main__":
    rospy.init_node('path_planner')

    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/planner/new_goal', PoseStamped, goal_callback)

    pub = rospy.Publisher('/planner/path', Path, queue_size=2)

    tf_buffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tf_buffer)

    cf_pose = None
    goal_pose = None

    main()

    rospy.spin()
