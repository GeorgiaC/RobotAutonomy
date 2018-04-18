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

        # cost from start to that node
        self.g_scores = dict()

        # cost of getting to final node from start node by passing through current node
        self.f_scores = dict()

        # already evaluated
        self.closed_set = Set()

        # not yet evaluated 
        self.open_set = Set()

        self.came_from = dict()

    def Plan(self, start_config, goal_config):
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # try: 
        #     self.planning_env.InitializePlot(goal_config)
        # except:
        #     print 'cant plot'

        # get dimension of environment 
        lims = self.planning_env.discrete_env.num_cells



        # total number of stats by multiplying all dimensions
        # total = int(np.prod(lims))

        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        # dictionary init, probably not the best way
        # for i in range (0, total):
        #     self.g_scores[i] = 1000
        #     self.f_scores[i] = 1000

        self.f_scores[start_id] = self.planning_env.ComputeHeuristicCost(start_id, goal_id)  # takes in id
        self.g_scores[start_id] = 0

        self.open_set.add (start_id)

        while len(self.open_set) > 0:

            # find the index with the lowest f_Score in open_set
            min = 1000
            for elem in self.open_set:

                try:
                    self.f_scores[elem]
                except: 
                    self.f_scores[elem] = 1000

                if self.f_scores[elem] <= min:
                    current = elem
                    min = self.f_scores[elem]

            if current == goal_id:
                return self.path(self.came_from, current)

            self.open_set.remove(current) 
            self.closed_set.add(current)

            neighbors = self.planning_env.GetSuccessors(current)

            for n in neighbors:

                # disregard if already visited
                if n in self.closed_set:
                    continue

                # have not been evaluated
                if n not in self.open_set:
                    self.open_set.add(n)

                    # plot everything
                    current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                    neighor_config = self.planning_env.discrete_env.NodeIdToConfiguration(n)

                temp_gscore = self.g_scores[current] + \
                              self.planning_env.ComputeDistance(current, n)

                # disregard a more costly path
                try:
                    self.g_scores[n]
                except:
                    self.g_scores[n] = 1000

                if temp_gscore >= self.g_scores[n]:
                    continue

                self.came_from[n] = current
                self.g_scores[n] = temp_gscore
                self.f_scores[n] = self.g_scores[n] + self.planning_env.ComputeHeuristicCost(n, goal_id)

        return 'aiya'  
<<<<<<< HEAD

    def path(self, came_from, current):  # current is an id

=======

    def path(self, came_from, current):  # current is an id

>>>>>>> origin/master
        #total_path = [self.planning_env.discrete_env.NodeIdToGridCoord(current)]
        total_path = []
        while current in came_from.keys():
            current = came_from[current]

            # convert current to x, y
            total_path.append(self.planning_env.discrete_env.NodeIdToConfiguration(current))
<<<<<<< HEAD

        total_path.reverse()

        # total_path.append([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43])
        print total_path
=======

        total_path.reverse()

        print total_path

>>>>>>> origin/master
        return total_path