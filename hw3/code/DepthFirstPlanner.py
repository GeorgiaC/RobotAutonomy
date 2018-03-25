import numpy as np
from collections import deque

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        
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

        # #create an array of nodes and give visited nodes their idx
        width = self.planning_env.discrete_env.num_cells[0]
        height = self.planning_env.discrete_env.num_cells[1]
        visit = np.zeros(width*height)
        visit[root_id] = 1
        self.nodes[start_id] = {}

        arrived = 0

        while Q :

            # print Q
            #take last node in list
            next_node = Q.pop()


            successors = self.planning_env.GetSuccessors(next_node)
            # print successors
            for id in successors:
                if visit[id] == 0:
                    # print id

                    #Mark visited node with it's index
                    idx += 1
                    visit[id] = idx

                    #add node's mother to dictionary and add node to queue
                    self.nodes[id] = next_node
                    # print self.nodes
                    Q.append(id) 

                    #plot edges
                    node_coord = self.planning_env.discrete_env.NodeIdToConfiguration(next_node)
                    id_coord = self.planning_env.discrete_env.NodeIdToConfiguration(id)
                    self.planning_env.PlotEdge(node_coord, id_coord)

                    if id == goal_id:
                        arrived = 1

            #if goal is reached, begin planning
            if arrived:
                break


        #Initialize plan
        plan = goal_config
        prev_node = goal_id

        while True:

            #break if you've reached the starting point
            if prev_node == start_id:
                break

            #find current node's mother node and set prev_node to mother
            mother = self.nodes[prev_node]
            prev_node = mother

            #add mother node to plan
            prev_config = self.planning_env.discrete_env.NodeIdToConfiguration(prev_node)

            plan = np.vstack((prev_config, plan))

            

        
        plan = np.vstack((start_config, plan))

        print plan
        return plan

