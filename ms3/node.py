#!/usr/bin/env python

import numpy as np


class Node:
    """ Node class for the nodes in the grid map. """
    def __init__(self, index, parent=None, position=np.zeros(3), yaw=0.0):
        self.index = index
        self.position = position
        self.yaw = yaw
        self.parent = parent
        self.cost2go = 0
        self.cost2come = 0
        self.cost = 0

