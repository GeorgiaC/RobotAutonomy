import numpy as np
import matplotlib.pyplot as pl
import time
from numpy import linalg as LA


class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 1.0], 
                               [-1, 0,  0, 0], 
                               [ 0, 1,  0, 0], 
                               [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #

        while (True):
            # generate a random config
            config = np.random.uniform(low=lower_limits, high=upper_limits, size=(2))
            H_trans = np.array([[1, 0, 0, config[0]],
                                [0, 1, 0, config[1]],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            self.robot.SetTransform(H_trans)

            if (self.robot.GetEnv().CheckCollision(self.robot) == False):
                break
        return config

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return np.linalg.norm(start_config - end_config)

    def can_connect(self, p1, p2):
        num_steps = 100

        delta = p2 - p1
        delta /= num_steps

        total_dist = LA.norm(p2 - p1)
        sub_dist = total_dist/num_steps

        i = 0

        for i in range (1, num_steps+1):
            H_trans = np.array([[1, 0, 0, delta[0]],
                                [0, 1, 0, delta[1]],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            self.robot.SetTransform(H_trans)

            
            dist = i*sub_dist
            new_delta = i*delta

            # print (dist)
            if (self.robot.GetEnv().CheckCollision(self.robot) == True):
            	return False

    	return True

    def Extend(self, start_config, end_config):
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        # num_steps = 100
        # tol = 0.1

        # delta = end_config - start_config
        # delta /= num_steps

        # # total_dist = LA.norm(start_config - end_config)
        # total_dist = self.ComputeDistance(start_config, end_config)
        # sub_dist = total_dist/num_steps
        # i = 0

        # # previous config might get returned
        # prev_config = start_config

        # while(True):
        #     H_trans = np.array([[1, 0, 0, delta[0]],
        #                         [0, 1, 0, delta[1]],
        #                         [0, 0, 1, 0],
        #                         [0, 0, 0, 1]])
        #     self.robot.SetTransform(H_trans)

        #     i += 1
        #     dist = i*sub_dist

        #     # print (dist)
        #     if (self.robot.GetEnv().CheckCollision(self.robot) == True) #  or total_dist-dist < tol):
        #         return prev_config

        # 	# update 
        #     prev_config += delta

        # return end_config

        can = self.can_connect(start_config, end_config)

        if (can == True):
        	return end_config
        else:
        	return None


    def ShortenPath(self, path, timeout=1.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start = time.time()

        n = len(path)
        print(n)

        while (True):
            i1, i2 = np.random.choice(n-1, 2)  # goes up to n-1, generate 2 numbers

            # see if those 2 points can connect
            if (self.can_connect(path[i1], path[i2]) == True):
            	# print ('can connect!')
            	# print (path[i1])
            	# print (path[i2])

            	# delete the points in between
                for it in range (i1+1, i2):
                	path = np.delete(path, it, 0)
                	print (path)

            curr_time = time.time()
            if (curr_time - start >= timeout):
            	break

        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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
        pl.pause(0.1)



