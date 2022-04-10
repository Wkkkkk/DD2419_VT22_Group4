# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import math
import json
import networkx as nx
import numpy as np

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, TransformStamped, Vector3
from localization import hcmatrix_from_transform

def marker_to_pose(marker):
    # p = Pose()
    position = marker['pose']['position']
    orientation = marker['pose']['orientation']

    p = Pose()
    p.position = Point(*position)
    roll, pitch, yaw = orientation
    (p.orientation.x,
     p.orientation.y,
     p.orientation.z,
     p.orientation.w) = quaternion_from_euler(math.radians(roll),
                                              math.radians(pitch),
                                              math.radians(yaw))
    # print ("p", type(p), p)
    return p




class Matcher:
    def __init__(self, static_marker_transforms):
        self.static_markers = {t.child_frame_id:t for t in static_marker_transforms}
        print(self.static_markers)

    def match(self, transform_list):
        """
        Kuhn-Munkres Algorithm
        :param marker_list: a list of transforms{geometry_msgs/TransformStamped} of detected markers
        :return: (dict) matching result， eg:{'aruco/detected1': 'aruco/marker1', 'aruco/detected2': 'aruco/marker2'}
        """
        graph = nx.Graph()
        for transform in transform_list:
            frame_id = transform.child_frame_id
            graph.add_node(frame_id, bipartite=0)
            for marker_frame_id, marker_transform in self.static_markers:
                graph.add_node(marker_frame_id, bipartite=1)
                score = self.calculate(transform, marker_transform)
                if score is not None:
                    graph.add_edge(frame_id, marker_frame_id, weight=score)
        match_set = nx.max_weight_matching(graph)
        res = dict()
        for (node_1, node_2) in match_set:
            if node_1.split('_')[0] == 'aruco/marker':
                node_1, node_2 = node_2, node_1
            res[node_1] = node_2
        return res


    def calculate(transform, static_transform):
        t1 = hcmatrix_from_transform(transform)
        t2 = hcmatrix_from_transform(static_transform)
        diff_abs = np.absolute(t1-t2)
        diff_sum = diff_abs.sum()

        return diff_sum