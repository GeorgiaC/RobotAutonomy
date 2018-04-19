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

        print 'starting config: ', start_config
        print 'goal config: ', goal_config

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
            min = 1000
            for elem in self.open_set:

                try:
                    self.f_scores[elem]
                except: 
                    self.f_scores[elem] = 1000

                if self.f_scores[elem] <= min:
                    current = elem
                    min = self.f_scores[elem]

            ###########
            #  done!  #
            ###########
            if current == goal_id:
                print 'goal found'
                return self.path(self.came_from, current)

            self.open_set.remove(current) 
            self.closed_set.add(current)

            neighbors = self.planning_env.GetSuccessors(current)

            for n in neighbors:  # iterate through all the action

                n_id = n[0]  # first item is the node id
                action = n[1]

                # disregard if already visited
                if n_id in self.closed_set:
                    continue

                # have not been evaluated
                if n_id not in self.open_set:
                    self.open_set.add(n_id)

                temp_gscore = self.g_scores[current] + self.planning_env.ComputeDistance(current, n_id)

                # disregard a more costly path
                try:
                    self.g_scores[n_id]
                except:
                    self.g_scores[n_id] = 1000

                if temp_gscore >= self.g_scores[n_id]:
                    continue

                # append the action class
                self.came_from[n_id] = [current, action] 
                self.g_scores[n_id] = temp_gscore
                self.f_scores[n_id] = self.g_scores[n_id] + self.planning_env.ComputeHeuristicCost(n_id, goal_id)

                 #plotting
                node_coord = self.planning_env.discrete_env.NodeIdToConfiguration(current)
                id_coord = self.planning_env.discrete_env.NodeIdToConfiguration(n_id)
                self.planning_env.PlotEdge(node_coord, id_coord)

        return 'aiya'  

    # def path(self, came_from, current):
    #     total_path = []
    #     count = 0

    #     for key, value in came_from.iteritems():
    #         action = value[1]
    #         total_path.append(action)

    #     total_path.reverse()
    #     # print total_path
    #     return total_path

    def path(self, came_from, current):  # current is an id

        total_path = []
        count = 0
        while current in came_from.keys():

            print count
            current, action = came_from[current]
            total_path.append(action)
            count += 1

        total_path.reverse()

        return total_path
