import logging, numpy, openravepy
import pdb

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        self.grasps = self.gmodel.grasps
        self.grasp_indices = self.gmodel.graspindices

        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################

        # base_pose, position of herb's body
        # grasp_config 7-DOF arm config
        order_grasps()

        pdb.set_trace()

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()

    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        for grasp in self.grasps_ordered:
            grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
            
        # sort!
        order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
        with self.robot:
            #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3,3]
                # for each contact
                G = np.array([]) #the wrench matrix

                arr = np.empty([6, len(contacts)])
                count = 0
                for c in contacts:
                    pos = c[0:3] - obj_position
                    dir = -c[3:] #this is already a unit vector
                    
                    ##############
                    # start TODO #
                    ##############
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
                    # end TODO

                # start TODO
                G = arr
                eigvals = np.sqrt(LA.eigvals(G.dot(np.transpose(G))))
                Q1 = np.min(eigvals)
                Q2 = np.sqrt(LA.det(G.dot(np.transpose(G))))
                ############
                # end TODO #
                ############

                return Q1 #change this