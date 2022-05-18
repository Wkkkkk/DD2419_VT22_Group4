#!/usr/bin/env python

import numpy as np


class Node:
    """ Node class for the nodes to compute a path in the grid map. """
    def __init__(self, index, parent=None, position=np.zeros(3), yaw=None):
        self.index = index  # Index of the node in the grid map
        self.position = position
        self.yaw = yaw
        self.parent = parent
        self.cost2go = 0    # Heuristic cost to go from the position of the node to the goal of the path
        self.cost2come = 0  # Cost to go from the start position of the path to the node position
        self.cost = 0   # cost2come + cost2go
