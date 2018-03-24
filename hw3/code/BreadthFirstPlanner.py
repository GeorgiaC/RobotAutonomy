import numpy as np
from collections import deque

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        

        plan = []
        self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        #Convert configs to Node_ids
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        root_id = start_id


        idx = 1 #order a node was visited in
        Q = deque([root_id]) #queue of nodes to expand

        #create an array of nodes and give visited nodes their idx
        width = self.planning_env.discrete_env.num_cells[0]
        height = self.planning_env.discrete_env.num_cells[1]
        visit = np.zeros(width*height)
        visit[root_id] = 1

        while Q :

            # print Q
            #take first node in list
            next_node = Q.popleft()

            if next_node == goal_id:
                break

            successors = self.planning_env.GetSuccessors(next_node)
            # print successors
            for id in successors:
                if visit[id] == 0:
                    # print id

                    #Mark visited node with it's index
                    idx += 1
                    visit[id] = idx
                    # #set resolution to 1 to view grid
                    # print np.reshape(visit, (10, 10))
                    #add new nodes to queue
                    Q.append(id) 

                    #plot edges
                    node_coord = self.planning_env.discrete_env.NodeIdToConfiguration(next_node)
                    id_coord = self.planning_env.discrete_env.NodeIdToConfiguration(id)
                    self.planning_env.PlotEdge(node_coord, id_coord)


        #Initialize plan
        plan = goal_config
        prev_node = goal_id

        best_idx = 10000
        prev_visit = 10000

        while True:

            #amoung a node's successors, find the one with the smalles idx
            successors = self.planning_env.GetSuccessors(prev_node)
            for id in successors:
                if visit[id] < prev_visit and visit[id] != 0:
                    best_idx = id
                    prev_visit = visit[id]
                    print best_idx

            #add smallest idx node to plan
            prev_node = best_idx
            prev_config = self.planning_env.discrete_env.NodeIdToConfiguration(prev_node)

            plan = np.vstack((prev_config, plan))

            if best_idx == start_id:
                break

        
        plan = np.vstack((start_config, plan))

        print plan
        return plan