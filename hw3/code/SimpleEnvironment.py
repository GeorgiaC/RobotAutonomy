import numpy as np
from numpy import linalg as LA
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
# import pdb

# import argparse, numpy, openravepy, time
# from HerbRobot import HerbRobot
# from SimpleRobot import SimpleRobot


class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        x, y = self.discrete_env.NodeIdToGridCoord(node_id)

        # pdb.set_trace()
        xlim, ylim = self.discrete_env.num_cells

        if (node_id < 0 or node_id > xlim*ylim-1):
            return None
        
        temp = [[x-1, y], [x+1, y], [x, y-1], [x, y+1]]
        for s in temp:
            if s[0] >= 0 and s[0] < xlim and s[1] >= 0 and s[1] < ylim:
                successors.append(self.discrete_env.GridCoordToNodeId(s))

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        s_config = np.asarray(self.discrete_env.NodeIdToConfiguration(start_id))
        e_config = np.asarray(self.discrete_env.NodeIdToConfiguration(end_id))

        dist = LA.norm(e_config-s_config)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        s_config = np.asarray(self.discrete_env.NodeIdToConfiguration(start_id))
        g_config = np.asarray(self.discrete_env.NodeIdToConfiguration(goal_id))
        cost = LA.norm(g_config-s_config)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        


# # test
# env = openravepy.Environment()
# robot = SimpleRobot(env)
# se = SimpleEnvironment(robot, 1.)

# # 1 
# print se.GetSuccessors(99)

# #2
# print se.ComputeDistance(0, 9)

# #3
# print se.ComputeHeuristicCost(0, 9)