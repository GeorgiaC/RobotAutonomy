import numpy as np
# import pdb

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
            lspace.append(np.linspace(self.lower_limits[i], self.upper_limits[i], num = n))

            config[i] = (lspace[i][coord[i]] + lspace[i][coord[i]-1])/2

        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 

        num_cells = np.asarray(self.num_cells)

        #create a grid that represents the numbered nodes
        grid = np.linspace(0, num_cells[0]*num_cells[1]-1, num_cells[0]*num_cells[1])
        grid = np.reshape(grid, (num_cells[0], num_cells[1]))

        #find the node at the given coordinate
        node_id = grid[coord[1]][coord[0]]

        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate

        num_cells = np.asarray(self.num_cells)

        #create a grid that represents the numbered nodes
        grid = np.linspace(0, num_cells[0]*num_cells[1]-1, num_cells[0]*num_cells[1])
        grid = np.reshape(grid, (num_cells[0], num_cells[1]))

        #find our position i nthe grid
        config = np.where(grid==node_id)

        #grid is organized in rows then columns
        coord = np.append(config[1], config[0])
        

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




