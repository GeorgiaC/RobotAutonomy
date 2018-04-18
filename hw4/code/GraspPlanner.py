import logging, openravepy
import pdb
import numpy as np
from numpy import linalg as LA

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner


    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

        if not self.gmodel.load():
            self.gmodel.autogenerate()

        self.grasps = self.gmodel.grasps
        self.grasp_indices = self.gmodel.graspindices

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################

        # base_pose, position of herb's body
        # grasp_config 7-DOF arm config

        # inverse reachibilty
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        if not self.irmodel.load():
            print "NO, it's gonna take forever!!!!!!!!!"
            self.irmodel.autogenerate()
            self.irmodel.load()
        else:
            print "IR available"

        self.order_grasps()
        best_grasp = self.grasps_ordered[0]
        Tgrasp = self.gmodel.getGlobalGraspTransform(best_grasp, collisionfree=True)

        _, sampler_func, _ = self.irmodel.computeBaseDistribution(Tgrasp)

        manipulator = self.robot.SetActiveManipulator('left_wam')

        # attempting to get the base pose from inverse reachability 
        poses, jointstate = sampler_func(69)
        
        for pose in poses:
            self.robot.SetTransform(pose)
            self.robot.SetDOFValues(*jointstate)

            angle = openravepy.axisAngleFromQuat(pose)
            base_config = np.array([pose[4], pose[5], angle[2]])
            grasp_config = manipulator.FindIKSolution(Tgrasp,
                                                      filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)

            # check for collision
            collision = self.check_collision(pose[4], pose[5])

            if grasp_config is not None and collision == False:
                # TODO check base collision
                print 'base config: ', base_config
                print 'grasp config: ', grasp_config
                return base_config, grasp_config

    def PlanToGrasp(self, obj):
        # start pose 
        start_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        print 'starting base pose: ', start_pose

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose

        print 'ending base pose: ', base_pose
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)
        # self.base_planner.planning_env.herb.get_starting_config(start_pose)  # added by Chris

        # Now plan the arm to the grasp configuration
        start_config = np.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)
        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()


    def check_collision(self, x, y):
        transform = np.array([[1, 0, 0, x],
                              [0, 1, 0, y],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.robot.SetTransform(transform)
        return self.robot.GetEnv().CheckCollision(self.robot)


    ############################
    #  from older assignments  # 
    ############################

    def order_grasps(self):
        self.grasps_ordered = np.asarray(self.grasps).copy()

        for grasp in self.grasps_ordered:
            grasp[self.grasp_indices.get('performance')] = self.eval_grasp(grasp)

        # sort!
        order = np.argsort(self.grasps_ordered[:,self.grasp_indices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
        with self.robot:
            #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts, finalconfig, mindist, volume = self.gmodel.testGrasp(grasp=grasp,
                                                                               translate=True,
                                                                               forceclosure=False)
                obj_position = self.gmodel.target.GetTransform()[0:3,3]
                G = np.array([]) #the wrench matrix
                arr = np.empty([6, len(contacts)])
                count = 0

                for c in contacts:
                    pos = c[0:3] - obj_position
                    dir = -c[3:] #this is already a unit vector

                    p = np.transpose(pos)
                    p = np.reshape(p, (3, 1))
                    n = np.transpose(dir)
                    n = np.reshape(n, (3, 1))

                    cross = np.cross(pos, dir)
                    cross = np.reshape(cross, (3, 1))

                    fill = np.concatenate((n, cross), axis=0)
                    fill = np.reshape(fill, (6,))

                    arr[:, count] = fill
                    count = count + 1

                G = arr
                eigvals = np.sqrt(LA.eigvals(G.dot(np.transpose(G))))
                Q1 = np.min(eigvals)
                Q2 = np.sqrt(LA.det(G.dot(np.transpose(G))))
                return Q1

            except openravepy.planning_error, e:
                return 0.00
