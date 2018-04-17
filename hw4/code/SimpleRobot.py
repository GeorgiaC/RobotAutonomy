import openravepy, time
import pdb
import numpy as np

class SimpleRobot(object):

    def __init__(self, env, robot):
        self.name = 'simple'
        self.robot = robot
        self.wheel_radius = 0.25
        self.wheel_distance = 0.5
        self.max_wheel_velocity = 1.0

    def get_starting_config(self, config):
        self.start_config = config

    def GetCurrentConfiguration(self):
        t = self.robot.GetTransform()
        aa = openravepy.axisAngleFromRotationMatrix(t)
        pose = [t[0,3], t[1,3], aa[2]]
        return np.array(pose)

    def SetCurrentConfiguration(self, config):
        
        transform = [[np.cos(config[2]), -np.sin(config[2]), 0, config[0]],
                     [np.sin(config[2]),  np.cos(config[2]), 0, config[1]],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self.robot.SetTransform(transform)

    def ConvertPlanToTrajectory(self, plan):
        # Create a trajectory and insert all points
        return plan

    def ExecuteTrajectory(self, traj, stepsize = 0.01):
        start_config = np.array(self.start_config)
        self.SetCurrentConfiguration(start_config)
        # Send the trajectory to the controller and wait for execution to complete
        offset = None
        for action in traj:
            # pdb.set_trace()
            config = self.GetCurrentConfiguration()

            for fconfig in action.footprint:
                new_config = np.asarray(fconfig).copy()
                new_config[:2] += config[:2]
                self.SetCurrentConfiguration(new_config)
                time.sleep(0.001)

