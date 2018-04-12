import openravepy
import numpy as np
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb
import HerbRobot as herb
import SimpleRobot as SR

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
        w_wheel = 0.5  # for now, the magnitude
        dt = 0.5

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


        pdb.set_trace()
         

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        # just check collision???
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        coord = self.discrete_env.NodeIdToCoord(node_id)
        coord_angle = coord[2]

        x_min = self.boundary_limits[0][0]
        y_min = self.boundary_limits[0][1]
        x_max = self.boundary_limits[1][0]
        y_max = self.boundary_limits[1][1]

        # iterate over all actions at orientation? seems like everything is orientation dependent...
        for action in self.actions[coord_angle]:

            # check current config + the transfrom from the list of actions
            curr_config = config + np.asarray(self.actions[coord_angle][i].footprint[-1])
            
            bounds_error = False

            # check out of bounds in terms of x and y, theta is taken care of
            if (curr_config[0] < x_min or curr_config[0] > x_max or 
                curr_config[1] < y_min or curr_config[1] > y_max):
                bounds_error = True

            # check collision
            x, y = curr_config[0:2]
            collision = self.check_collision(x, y)

            # if all good
            if (collision == False and bounds_error == False):
                successors.append(action)

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

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost


if __name__ == "__main__":
    env = openravepy.Environment()
    PR2robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
    robot = SR.SimpleRobot(env, PR2robot)
    s = SimpleEnvironment(robot, resolution=np.array([0.05, 0.05, 0.05]))
    s.ConstructActions()
