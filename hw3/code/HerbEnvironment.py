import numpy as np
from DiscreteEnvironment import DiscreteEnvironment
import pdb
from numpy import linalg as LA

import argparse, numpy, openravepy, time
from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        self.upper_coord = [x - 1 for x in self.discrete_env.num_cells]

        self.upper_config = self.discrete_env.GridCoordToConfiguration(self.upper_coord)
        for idx in range(len(self.upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        print (self.lower_limits)
        print (self.upper_limits)

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 0.7], 
                               [-1, 0,  0, 0], 
                               [ 0, 1,  0, 0], 
                               [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = np.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []
        temp = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        herb_coord = self.discrete_env.NodeIdToGridCoord(node_id)  # 7D
        limits = self.discrete_env.num_cells

        # all possibilities
        for i in range (0, len(herb_coord)):
            temp_coord = np.asarray(herb_coord)
            temp_coord[i] += 1
            temp.append(temp_coord.tolist())
            temp_coord = np.asarray(herb_coord)
            temp_coord[i] -= 1
            temp.append(temp_coord.tolist())

        # upper
        # [ 0.52359878 -1.97222205 -2.74016693 -0.87266463 -4.79965544 -1.57079633 -3.00196631]
        # [ 5.75958653  1.97222205  2.74016693  3.14159265  1.30899694  1.57079633 3.00196631]

        # filter
        for coord in temp:
            flag = True

            # check in continuous space
            config = self.discrete_env.GridCoordToConfiguration(coord)
            for idx, con_num in enumerate(config):
                if con_num < self.lower_limits[idx] or con_num > self.upper_limits[idx]:
                    flag = False

            # check each number for negatives and out of range
            for i in range (0, len(coord)):

                number = coord[i]
                # pdb.set_trace()
                if number < 0 or number >= limits[i]:  # must be one less due to indexing
                    flag = False

            with self.robot:


            # test = [4.4505895925855405, -1.6904760469316509, 0.27401669256310957, 1.8871372537188689, -0.98174770424681013, 0.78539816339744806, -1.7511470161676443]
            # self.robot.SetActiveDOFValues(test)

                self.robot.SetActiveDOFValues(self.discrete_env.GridCoordToConfiguration(coord))
                if (self.robot.GetEnv().CheckCollision(self.robot) or \
                    self.robot.CheckSelfCollision()):
                    # print 'collision'
                    flag = False
            # pdb.set_trace()

            if flag:
                successors.append(self.discrete_env.GridCoordToNodeId(coord))
                # print successors


        # pdb.set_trace()
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
        # cost = 100*LA.norm(g_config-s_config)

        
        return cost

# # test
# env = openravepy.Environment()
# robot = HerbRobot(env, 'right')
# se = HerbEnvironment(robot, 0.1)

# se.GetSuccessors(1)