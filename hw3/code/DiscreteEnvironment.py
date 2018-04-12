import numpy as np
import pdb

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #

        # lspace = []
        # ind = []
        # node_id = 0
        # for i in range (0, self.dimension):
        #     n = int((self.upper_limits[i]-self.lower_limits[i])/self.resolution)+1
        #     lspace.append(np.linspace(self.lower_limits[i], self.upper_limits[i], num = n))
        #     ind.append(np.digitize(np.array([config[i]]), lspace[i])[0]-1)

        #     node_id += 10**(self.dimension-1-i)*ind[0]

        # print lspace
        # return node_id
        temp = self.ConfigurationToGridCoord(config)
        return self.GridCoordToNodeId(temp)

    def NodeIdToConfiguration(self, ind):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        temp = self.NodeIdToGridCoord(ind)
        return self.GridCoordToConfiguration(temp)
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #

        lspace = []
        ind = []
        coord = [0] * self.dimension

        for i in range (0, self.dimension):
            n = int((self.upper_limits[i]-self.lower_limits[i])/self.resolution)+1
            lspace.append(np.linspace(self.lower_limits[i], self.upper_limits[i], num = n))
            ind.append(np.digitize(np.array([config[i]]), lspace[i])[0]-1)

        return ind

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        lspace = []
        ind = []

        for i in range (0, self.dimension):
            n = int((self.upper_limits[i]-self.lower_limits[i])/self.resolution)+1

            # pdb.set_trace()
            lspace.append(np.linspace(self.lower_limits[i], self.upper_limits[i], num = n))

            if coord[i] + 1 < n:
                config[i] = (lspace[i][coord[i]] + lspace[i][coord[i]+1])/2
            else:
                config[i] = lspace[i][coord[i]]

        return config

    def GridCoordToNodeId(self,coord):
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        # coord.tolist()
        coord = map(int, coord)
        # pdb.set_trace()
        node_id = np.ravel_multi_index(coord, self.num_cells, order='F')



        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        num_cells = np.asarray(self.num_cells)

        coord = np.unravel_index(np.asarray(node_id).astype(int), num_cells, order='F')

        return coord
        
        
# # testing
# env = DiscreteEnvironment(resolution=0.2, lower_limits=[0,0], upper_limits=[1,1])
# # id = env.ConfigurationToNodeId(config)
# # print id


# # 1 test
# hi = env.ConfigurationToGridCoord([0.26, 0.64])
# print 'test 1: ', hi

# # 2 test
# hi = env.GridCoordToConfiguration([1, 3])
# print 'test 2: ', hi

# # 3 test
# hi = env.GridCoordToNodeId([1,3])
# print 'test 3: ', hi

# # 4 test
# hi = env.NodeIdToGridCoord(16)
# print 'test 4: ', hi

# # 5 test
# hi = env.ConfigurationToNodeId([0.26, 0.64])
# print 'test 5: ', hi

# # 6 test
# hi = env.NodeIdToConfiguration(16)
# print 'test 6: ', hi



