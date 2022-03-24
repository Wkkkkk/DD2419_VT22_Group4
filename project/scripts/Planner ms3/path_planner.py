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
from tf.transformations import quaternion_from_euler
from grid_map import GridMap
from node import Node


def pose_callback(msg):
    """ Retrieves the current pose of the drone in odom frame. """
    global current_pose
    current_pose = transform2map(msg)


def transform2map(m):
    """ Transforms the drone pose from odom frame to map frame """
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


class Planner:
    def __init__(self, goal_pose, world):
        self.grid = GridMap(0.1, world)

        self.start, self.goal = self.initialize_planning(goal_pose)
        self.grid[self.start.index] = self.start
        self.grid[self.goal.index] = self.goal

        X = self.grid.dim[0]
        Y = self.grid.dim[1]
        self.neighbours = lambda x, y: [self.grid[(i, j)] for i in range(x-1, x+2) for j in range(y-1, y+2)
                                        if ((0 <= i <= X) and (0 <= j <= Y) and i != x and j != y
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
        tol = 0.1
        setpoints = []
        goal = pose_stamped(self.goal.position, self.goal.yaw)
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
            if abs(p.yaw - node.yaw) > tol:
                setpoints.append(pose_stamped(node.position, p.yaw))
            p.position[2] = self.start.position[2]
            setpoints.append(pose_stamped(p.position, p.yaw))
            node = p
        if self.start.parent is not None:
            if self.start.parent.yaw != self.start.yaw:
                setpoints.append(pose_stamped(self.start.position, self.start.parent.yaw))
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
            for e in open_set:
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
            path_publisher(setpoints)
            return setpoints
        else:
            rospy.loginfo("Could not find a trajectory!")
            return None

    def initialize_planning(self, goal_pose):
        global current_pose
        current_pose = None

        while not current_pose:
            continue

        root_pose = current_pose

        root_yaw = get_yaw(root_pose.pose.orientation)
        root_pos = np.array([root_pose.pose.position.x, root_pose.pose.position.y, root_pose.pose.position.z])
        start_pos = root_pos.copy()

        goal_yaw = get_yaw(goal_pose.pose.orientation)
        goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])
        tol = 1e-1

        if root_pos[2] < tol:
            start_pos[2] = goal_pos[2]

        root = Node(self.grid.convert_to_index(root_pos), None, root_pos, root_yaw)

        if root_pos[2] < tol:
            start = Node(self.grid.convert_to_index(start_pos), root, start_pos, root_yaw)
        else:
            start = root
        goal = Node(self.grid.convert_to_index(goal_pos), None, goal_pos, goal_yaw)

        return start, goal


def get_yaw(q):
    return atan2(2 * (q.w * q.z + q.x * q.y),
                 1 - 2 * (q.y * q.y + q.z * q.z))


def yaw2quaternion(yaw):
    return quaternion_from_euler(0.0, 0.0, yaw)


def pose_stamped(position, yaw):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    q = yaw2quaternion(yaw)
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


def main(argv=sys.argv):

    args = rospy.myargv(argv=argv)
    with open(args[1], 'rb') as f:
        world = json.load(f)

    #while not rospy.is_shutdown():
    goal_pose = pose_stamped(np.array([0.0, 2.3, 0.5]), 0.0)
    A = Planner(goal_pose, world)
    A.run()


if __name__ == "__main__":
    rospy.init_node('path_planner')

    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)

    pub = rospy.Publisher('/planner/path', Path, queue_size=2)

    tf_buffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tf_buffer)

    current_pose = None

    main()

    rospy.spin()
