from heapq import heappush, heappop
import heapq
import pdb
from collections import OrderedDict
from sets import Set

import numpy as np


class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

        self.nodes = dict()
        self.g_scores = dict()
        self.f_scores = dict()
        self.came_from = dict()

        self.closed_set = Set()
        self.open_set = Set()


    def Plan(self, start_config, goal_config):
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        lims = self.planning_env.discrete_env.num_cells
        total = int(np.prod(lims))

        goal_id = self.planning_env.discrete_env.GridCoordToNodeId(goal_config)
        start_id = self.planning_env.discrete_env.GridCoordToNodeId(start_config)

        # dictionary init, probably not the best way
        for i in range (0, total):
            self.g_scores[i] = 1000
            self.f_scores[i] = 1000
        
        # pdb.set_trace()

        self.f_scores[start_id] = self.planning_env.ComputeHeuristicCost(start_config.tolist(), goal_config.tolist())
        self.g_scores[start_id] = 0

        self.open_set.add (start_id)

        while len(self.open_set) > 0:

            # current = sorted(self.f_scores, key=lambda k: self.f_scores[k])[0]  # find the index with the lowest f_Score
            min = 1000
            for elem in self.open_set:
                if self.f_scores[elem] <= min:
                    current = elem
                    min = self.f_scores[elem]

            if current == goal_id:
                return self.path(self.came_from, current)
            # pdb.set_trace()

            self.open_set.remove(current)
            self.closed_set.add(current)

            neighbors, _ = self.planning_env.GetSuccessors(current)

            for n in neighbors:
                if n in self.closed_set:
                    continue

                if n not in self.open_set:
                    self.open_set.add(n)

                temp_gscore = self.g_scores[current] + self.planning_env.ComputeDistance(current, n)
                if temp_gscore >= self.g_scores[n]:
                    continue

                self.came_from[n] = current
                self.g_scores[n] = temp_gscore
                self.f_scores[n] = self.g_scores[n] + self.planning_env.ComputeHeuristicCost(n, goal_id)

        return -1  

        # plan.append(start_config)
        # plan.append(goal_config)

        return plan

    def path(self, came_from, current):
        current = self.planning_env.discrete_env.NodeIdToGridCoord(current)
        total_path = [current]

        while current in came_from:
            current = came_from[current]

            # convert current to x, y
            current = self.planning_env.discrete_env.NodeIdToGridCoord(current)
            # pdb.set_trace()
            total_path.append(current)

        pdb.set_trace()
        return total_path



