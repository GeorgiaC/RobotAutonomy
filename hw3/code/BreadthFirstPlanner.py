import numpy as np
from collections import deque
# from sets import Set
# import pdb

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        # self.que = deque([], maxlen = 1000)
        # self.set = Set()
        # self.dict = dict()
        
    def Plan(self, start_config, goal_config):
        
        # start_config = np.ndarray.tolist(start_config)
        # goal_config = np.ndarray.tolist(goal_config)

        plan = []
        self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        print goal_id
        # init
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        root_id = start_id
        print root_id

        # self.dict[root_id] = (None, None)
        # self.que.append(root_id)

        # while not len(self.que) <= 0:

        #     sub_root_id = self.que.pop()

        #     if (sub_root_id == goal_id):
        #         pdb.set_trace()
        #         return self.path(sub_root_id, self.dict)

        #     temp1, temp2 = self.planning_env.GetSuccessors(sub_root_id)

        #     for child, action in zip (temp1, temp2):
        #         if child in self.set:
        #             continue

        #         if child not in self.que:
        #             self.dict[child] = (sub_root_id, action)
        #             self.que.append(child)

        #     self.set.add(sub_root_id)
   
        # return -1
        idx = 1
        Q = deque([root_id])
        Ids = np.array([root_id])
        Idx = np.array([idx])

        width = self.planning_env.discrete_env.num_cells[0]
        height = self.planning_env.discrete_env.num_cells[1]
        visit = np.zeros(width*height)
        visit[root_id] = 1

        while Q :

            # print Q
            next_node = Q.popleft()
            print np.reshape(visit, [10, 10])

            if next_node == goal_id:
                break

            successors = self.planning_env.GetSuccessors(next_node)
            # print successors
            for id in successors:
                if visit[id] == 0:
                    # print id

                    idx += 1
                    visit[id] = idx
                    Q.append(id)
                    Ids = np.append(Ids, id)
                    Idx = np.append(Idx, idx)

                    #plot edges
                    node_coord = self.planning_env.discrete_env.NodeIdToConfiguration(next_node)
                    id_coord = self.planning_env.discrete_env.NodeIdToConfiguration(id)
                    self.planning_env.PlotEdge(node_coord, id_coord)



        plan = goal_config
        prev_node = goal_id

        best_idx = 10000
        prev_visit = 10000

        while True:

            successors = self.planning_env.GetSuccessors(prev_node)

            for id in successors:
                if visit[id] < prev_visit and visit[id] != 0:
                    best_idx = id
                    prev_visit = visit[id]
                    print best_idx

            prev_node = best_idx
            prev_config = self.planning_env.discrete_env.NodeIdToConfiguration(prev_node)
            print plan
            print prev_config
            plan = np.vstack((prev_config, plan))

            if best_idx == start_id:
                break

        
        plan = np.vstack((start_config, plan))

        print plan
        return plan



    # def path(self, state, dict):
    #     action_list = []
    #     state_list = []

    #     while True:
    #         row = dict[state]
    #         if len(row) == 2:
    #             state = row[0]
    #             action = row[1]
    #             action_list.append(action)
    #             state_list.append(state)
    #         else:
    #             break

    #     return state_list.reverse()