import numpy as np
from collections import deque
from sets import Set
import pdb

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.que = deque([], maxlen = 1000)
        self.set = Set()
        self.dict = dict()
        
    def Plan(self, start_config, goal_config):
        
        # start_config = np.ndarray.tolist(start_config)
        # goal_config = np.ndarray.tolist(goal_config)

        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        goal_id = self.planning_env.discrete_env.GridCoordToNodeId(goal_config)

        # init
        root_conf = start_config
        root_id = self.planning_env.discrete_env.GridCoordToNodeId(root_conf)
        self.dict[root_id] = (None, None)
        self.que.append(root_id)

        while not len(self.que) <= 0:

            sub_root_id = self.que.pop()

            if (sub_root_id == goal_id):
                pdb.set_trace()
                return self.path(sub_root_id, self.dict)

            temp1, temp2 = self.planning_env.GetSuccessors(sub_root_id)

            for child, action in zip (temp1, temp2):
                if child in self.set:
                    continue

                if child not in self.que:
                    self.dict[child] = (sub_root_id, action)
                    self.que.append(child)

            self.set.add(sub_root_id)
   
        return -1
        # idx = 1
        # Q = [root_id]
        # Ids = np.array([root_id])
        # Idx = np.array([idx])


        # while True:

        #     next_node = Q.pop()
        #     print Q

        #     if next_node == goal_id:
        #         break

        #     successors = self.planning_env.GetSuccessors(next_node)
        #     for id in successors:
        #         if id not in Q:
        #             idx += idx
        #             Q.append(id)
        #             Ids = np.append(Ids, id)
        #             Idx = np.append(Idx, idx)
        #             node_coord = self.planning_env.discrete_env.NodeIdToGridCoord(next_node)
        #             id_coord = self.planning_env.discrete_env.NodeIdToGridCoord(id)
        #             self.planning_env.PlotEdge(node_coord, id_coord)


        # plan = [root_id]

        # while true:

        #     next_node = path[len(path)]

        #     if next_node == goald_id:
        #         break

        #     successors = self.planning_env.GetSuccessors(next_node)
        #     path.append(np.amin(successors))

        # return plan



    def path(self, state, dict):
        action_list = []
        state_list = []

        while True:
            row = dict[state]
            if len(row) == 2:
                state = row[0]
                action = row[1]
                action_list.append(action)
                state_list.append(state)
            else:
                break

        return state_list.reverse()