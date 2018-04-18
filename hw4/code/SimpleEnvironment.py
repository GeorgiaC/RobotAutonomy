import openravepy
import numpy as np
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb
import HerbRobot as herb
import SimpleRobot as SR

from numpy import linalg as LA

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -np.pi], [5., 5., np.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [np.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * np.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*np.array([xdot, ydot, tdot])
            if config[2] > np.pi:
                config[2] -= 2.*np.pi
            if config[2] < -np.pi:
                config[2] += 2.*np.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()
        

    def ConstructActions(self):
        L = 0.5
        r = 0.2
        w_wheel = 1  # for now, the magnitude
        dt = self.discrete_env.resolution[0]/.2 #time to traverse one cell

        # calculate dt so that the robot moves 1 block

        # Actions is a dictionary that maps orientation of the robot to
        # an action set
        self.actions = dict()
 
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        # number of times the angle gets sliced into
        for idx in range(int(self.discrete_env.num_cells[2])):  
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)
            start_config = np.asarray(start_config)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            # go forwards
            control_f = Control(w_wheel, w_wheel, dt)
            fp_f = self.GenerateFootprintFromControl(start_config, control_f)
            action_f = Action(control_f, fp_f)
            self.actions[idx].append(action_f)

            # go backwards
            control_b = Control(-w_wheel, -w_wheel, dt)
            fp_b = self.GenerateFootprintFromControl(start_config, control_b)
            action_b = Action(control_b, fp_b)
            self.actions[idx].append(action_b)

            # turn 90 degrees
            angle_90 = np.pi/2
            delta_t = angle_90*L/(2*r*w_wheel)
            wee = angle_90*L/(2*r*dt)

            # to the left
            control_90l = Control(-w_wheel, w_wheel, delta_t)
            fp_90l = self.GenerateFootprintFromControl(start_config, control_90l) 
            action_90l = Action(control_90l, fp_90l)
            self.actions[idx].append(action_90l)

            # to the right
            control_90r = Control(w_wheel, -w_wheel, delta_t)
            fp_90r = self.GenerateFootprintFromControl(start_config, control_90r) 
            action_90r = Action(control_90r, fp_90r)
            self.actions[idx].append(action_90r)


    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        # make sure you dont add the theta, just going forward!!!

        # just check collision???
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        coord_angle = coord[2]

        x_min = self.boundary_limits[0][0]
        y_min = self.boundary_limits[0][1]
        t_min = self.boundary_limits[0][2]
        x_max = self.boundary_limits[1][0]
        y_max = self.boundary_limits[1][1]
        t_max = self.boundary_limits[1][2]

        count = 0  # indicating whether or not it's a translation or turn

        # iterate over all actions at orientation? seems like everything is orientation dependent...
        for action in self.actions[coord_angle]:

            bounds_error = False

            for fp in action.footprint:

                dx = fp[0]
                dy = fp[1]
                the = fp[2]

                curr_config = np.array([config[0]+dx, config[1]+dy, the])
            
                # check out of bounds in terms of x and y
                if (curr_config[0] < x_min or curr_config[0] > x_max or 
                    curr_config[1] < y_min or curr_config[1] > y_max or
                    curr_config[2] < t_min or curr_config[2] > t_max ):

                    bounds_error = True
                    continue

                # check collision
                x, y = curr_config[0:2]
                collision = self.check_collision(x, y)
                if collision == True:
                    print "collision"
                    continue

            # if all good
            if (collision == False and bounds_error == False):

                final_footprint = action.footprint[-1]
                dest_config = np.asarray([config[0], config[1], 0]) + final_footprint
                dest_id = self.discrete_env.ConfigurationToNodeId(dest_config)

                # im not adding initial config to the list of action
                successors.append([dest_id, action])

            count += 1  # indicator

        return successors


    def check_collision(self, x, y):
        transform = np.array([[1, 0, 0, x],
                              [0, 1, 0, y],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.robot.SetTransform(transform)
        return self.robot.GetEnv().CheckCollision(self.robot)


    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        start_pos = np.asarray(start_config[0:2])
        end_pos = np.asarray(end_config[0:2])

        dist = LA.norm(start_pos - end_pos)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)

        # weighting factor
        w = np.array([1, 1, 0.1])
        start_config = start_config*w
        end_confg = end_config*w 

        cost = LA.norm(np.asarray(start_config) - np.asarray(end_config))
        
        return cost

    # def ComputeDistance(self, start_id, end_id):

    #     start_coord = np.asarray(self.discrete_env.NodeIdToConfiguration(start_id))
    #     end_coord = np.asarray(self.discrete_env.NodeIdToConfiguration(end_id))
    #     dist_x = np.linalg.norm(start_coord[:2] - end_coord[:2])
    #     dist_theta = abs(start_coord[2] - end_coord[2])
    #     dx = np.sqrt(self.discrete_env.resolution[0] ** 2 + self.discrete_env.resolution[1] ** 2)
    #     dtheta = np.pi / 2
    #     return dist_x/dx + dist_theta/dtheta

    # def ComputeHeuristicCost(self, start_id, goal_id):

    #     cost = 0

    #     cost = self.ComputeDistance(start_id, goal_id)  
    #     return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.boundary_limits[0][0], self.boundary_limits[1][0]])
        pl.ylim([self.boundary_limits[0][1], self.boundary_limits[1][1]])
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


# if __name__ == "__main__":
#     env = openravepy.Environment()
#     PR2robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
#     robot = SR.SimpleRobot(env, PR2robot)
#     s = SimpleEnvironment(robot, resolution=np.array([0.05, 0.05, 0.05]))
#     s.ConstructActions()
#     l = s.GetSuccessors(100)
