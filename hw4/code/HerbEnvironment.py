import numpy
import random
import time 
import matplotlib.pyplot as pl

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        # table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        # self.robot.GetEnv().Add(table)

        # table_pose = numpy.array([[ 0, 0, -1, 0.6], 
        #                           [-1, 0,  0, 0], 
        #                           [ 0, 1,  0, 0], 
        #                           [ 0, 0,  0, 1]])
        # table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        print(lower_limits, upper_limits)
        config = [0] * len(self.robot.GetActiveDOFIndices())
        while (True):
            # generate a random config
            for i in range(len(config)):
                config[i] = (upper_limits[i]-lower_limits[i])*random.random() + lower_limits[i]  
            self.robot.SetActiveDOFValues(config)

            if (self.robot.GetEnv().CheckCollision(self.robot) == False and self.robot.CheckSelfCollision()==False):
        	    break
        print(config)
        return numpy.array(config)


    
    def ComputeDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return numpy.linalg.norm(start_config-end_config)

    def can_connect(self, p1, p2):
        num_steps = 100

        delta = p2 - p1
        delta /= num_steps

        total_dist = numpy.linalg.norm(p2 - p1)
        sub_dist = total_dist/num_steps

        i = 0

        for i in range (1, num_steps+1):
            self.robot.SetActiveDOFValues(p1 + i*delta)            
            dist = i*sub_dist
            new_delta = i*delta

            # print (dist)
            if (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()):
                return p1+(i-1)*delta

        return p1+(i-1)*delta

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        return self.can_connect(start_config,end_config)
        # can = self.can_connect(start_config, end_config)

        # if (can == True):
        #     return end_config
        # else:
        #     return None

    def can_connect_II(self, p1, p2):
        num_steps = 100

        delta = p2 - p1
        delta /= num_steps

        total_dist = numpy.linalg.norm(p2 - p1)
        sub_dist = total_dist/num_steps

        i = 0

        for i in range (1, num_steps+1):
            self.robot.SetActiveDOFValues(p1 + i*delta)            
            dist = i*sub_dist
            new_delta = i*delta

            # print (dist)
            if (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()):
                return False

        return True
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start = time.time()

        n = len(path)

        while (True):
            i1, i2 = numpy.random.choice(n-1, 2)  # goes up to n-1, generate 2 numbers

            # see if those 2 points can connect
            if (self.can_connect_II(path[i1], path[i2]) == True):
                # print ('can connect!')
                # print (path[i1])
                # print (path[i2])

                # delete the points in between
                for it in range (i1+1, i2):
                    path = numpy.delete(path, it, 0)
                    print (path)

            curr_time = time.time()
            if (curr_time - start >= timeout):
                break

        return path

    # def InitializePlot(self, goal_config):
    #     self.fig = pl.figure()
    #     lower_limits, upper_limits = self.boundary_limits
    #     pl.xlim([lower_limits[0], upper_limits[0]])
    #     pl.ylim([lower_limits[1], upper_limits[1]])
    #     pl.plot(goal_config[0], goal_config[1], 'gx')

    #     # Show all obstacles in environment
    #     for b in self.robot.GetEnv().GetBodies():
    #         if b.GetName() == self.robot.GetName():
    #             continue
    #         bb = b.ComputeAABB()
    #         pl.plot([bb.pos()[0] - bb.extents()[0],
    #                  bb.pos()[0] + bb.extents()[0],
    #                  bb.pos()[0] + bb.extents()[0],
    #                  bb.pos()[0] - bb.extents()[0],
    #                  bb.pos()[0] - bb.extents()[0]],
    #                 [bb.pos()[1] - bb.extents()[1],
    #                  bb.pos()[1] - bb.extents()[1],
    #                  bb.pos()[1] + bb.extents()[1],
    #                  bb.pos()[1] + bb.extents()[1],
    #                  bb.pos()[1] - bb.extents()[1]], 'r')
        
    #     pl.ion()
    #     pl.show()
        
    # def PlotEdge(self, sconfig, econfig):
    #     pl.plot([sconfig[0], econfig[0]],
    #             [sconfig[1], econfig[1]],
    #             'k.-', linewidth=2.5)
    #     pl.draw()
    #     pl.pause(0.1)
