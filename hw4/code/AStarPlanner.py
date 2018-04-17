from heapq import heappush, heappop
import heapq
import pdb
from collections import OrderedDict
from sets import Set
from numpy import linalg as LA

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

        try: 
            self.planning_env.InitializePlot(goal_config)
        except:
            print 'cant plot'

        # get dimension of environment 
        lims = self.planning_env.discrete_env.num_cells

        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        self.f_scores[start_id] = self.planning_env.ComputeHeuristicCost(start_id, goal_id)  # takes in id
        self.g_scores[start_id] = 0

        self.open_set.add (start_id)

        while len(self.open_set) > 0:
            # find the index with the lowest f_Score in open_set
            min = 10000
            for elem in self.open_set:

                try:
                    self.f_scores[elem]
                except: 
                    self.f_scores[elem] = 1000

                if self.f_scores[elem] <= min:
                    current = elem
                    min = self.f_scores[elem]

            if current == goal_id:
                print 'goal found'
                return self.path(self.came_from, current)

            self.open_set.remove(current) 
            self.closed_set.add(current)

            print 'current: ', current
            # print 'config: ', self.planning_env.discrete_env.NodeIdToConfiguration(current)

            neighbors = self.planning_env.GetSuccessors(current)

            for n in neighbors:  # iterate through all the action, should be 4 of them,

                n_id = n[0]  # first item is the node id
                control = n[1].control
                footprint = n[1].footprint
                action_config = footprint[-1]

                # disregard if already visited
                if n_id in self.closed_set:
                    continue

                # have not been evaluated
                if n_id not in self.open_set:
                    self.open_set.add(n_id)

                    current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                    distance = LA.norm(np.asarray(action_config[0:2]) - np.asarray(current_config[0:2]))

                temp_gscore = self.g_scores[current] + distance
                              # self.planning_env.ComputeDistance(current, n)

                # disregard a more costly path
                try:
                    self.g_scores[n_id]
                except:
                    self.g_scores[n_id] = 1000

                if temp_gscore >= self.g_scores[n_id]:
                    continue

                h_dist = LA.norm(np.asarray(goal_config) - np.asarray(current_config))
                self.came_from[n_id] = current
                # self.came_from[n_id] = control
                self.g_scores[n_id] = temp_gscore
                # self.f_scores[n] = self.g_scores[n] + self.planning_env.ComputeHeuristicCost(n, goal_id)
                self.f_scores[n_id] = self.g_scores[n_id] + h_dist

        return 'aiya'  

    def path(self, came_from, current):  # current is an id

        total_path = []
        while current in came_from.keys():
            current = came_from[current]

            # convert current to x, y
            total_path.append(self.planning_env.discrete_env.NodeIdToConfiguration(current))

        total_path.reverse()

        print total_path
        return total_path