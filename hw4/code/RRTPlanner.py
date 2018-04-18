import numpy as np
from RRTTree import RRTTree
import SimpleEnvironment

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 1):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # forward pass
        while (True):
            # generate point
            new_p = self.planning_env.GenerateRandomConfiguration()

			# get id and distance of a node closest
            (v_id, n_vertex) = tree.GetNearestVertex(new_p) 

			# see if the closet point to new point can connect
            potential = self.planning_env.Extend(n_vertex, new_p)

            # check if you can connect
            if (potential != None):
                # check distance 
                dist = self.planning_env.ComputeDistance(potential, goal_config)

                # add to tree class
                temp = tree.AddVertex(potential)
                tree.AddEdge(v_id, temp)

                self.planning_env.PlotEdge(n_vertex, potential)

            if (self.planning_env.ComputeDistance(potential, goal_config) <= epsilon):
                break

        # back prop 
        plan.append(goal_config)

        get_keys = tree.edges.keys()
        tree_key = get_keys[len(tree.edges)-1]

        while(True):

            plan.append(tree.vertices[tree_key])
            tree_key = tree.edges.get(tree_key)

            if(tree_key == 0):
                break;
        
        # append starting point and reverse
        plan.append(start_config)  
        plan.reverse()

        return plan
