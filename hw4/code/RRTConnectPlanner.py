import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.5):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        #     self.planning_env.InitializePlot(goal_config)
        # # TODO: Here you will implement the rrt connect planner
        # #  The return path should be an array
        # #  of dimension k x n where k is the number of waypoints
        # #  and n is the dimension of the robots configuration space

        # initial conditions
        ftree.AddVertex(start_config)
        rtree.AddVertex(goal_config)

        while (True):

            # generate 2 points, one for each side 
            new_f = self.planning_env.GenerateRandomConfiguration()
            
            # find nearest vertex
            f_id, f_vert = ftree.GetNearestVertex(new_f) 

            # try to extend
            pot_f = self.planning_env.Extend(f_vert, new_f)

            if (pot_f != None):
                dist_f = self.planning_env.ComputeDistance(pot_f, start_config)

                # add tree elements
                temp = ftree.AddVertex(pot_f)
                ftree.AddEdge(f_id, temp)

                # self.planning_env.PlotEdge(f_vert, pot_f)


            new_r = self.planning_env.GenerateRandomConfiguration()
            r_id, r_vert = rtree.GetNearestVertex(new_r)
            pot_r = self.planning_env.Extend(r_vert, new_r)

            if (pot_r != None):
                dist_r = self.planning_env.ComputeDistance(pot_r, goal_config)

                # add tree elements
                temp = rtree.AddVertex(pot_r)
                rtree.AddEdge(r_id, temp)

                # self.planning_env.PlotEdge(r_vert, pot_r)


            if (dist_f <= epsilon and dist_r <= epsilon) or self.planning_env.Extend(f_vert, r_vert)!= None:
                break

        # back prop, from middle to the front
        get_keys = ftree.edges.keys()
        tree_key = get_keys[len(ftree.edges)-1]

        while(True):

            plan.append(ftree.vertices[tree_key])
            tree_key = ftree.edges.get(tree_key)

            if(tree_key == 0):
                break;

        plan.append(start_config)
        plan.reverse()  # now it's from beginning to the middle

        # from the middle to end, no need to reverse
        get_keys = rtree.edges.keys()
        tree_key = get_keys[len(rtree.edges)-1]

        while (True):
            plan.append(rtree.vertices[tree_key])
            tree_key = rtree.edges.get(tree_key)

            if(tree_key == 0):
                break;
        
        plan.append(goal_config)

        return plan

