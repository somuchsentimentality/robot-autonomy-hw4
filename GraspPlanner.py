import logging, openravepy, math
import numpy as np

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        
        

    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        gmodel = self.gmodel # local copy
        if not self.gmodel.load():
            self.gmodel.autogenerate()
        else:
            print "loaded grasp model"
        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps

        print "ordering grasps..."
        self.order_grasps()
        print self.grasps_ordered

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        print "Finding base pose and grasp..."
        validgrasp=self.grasps_ordered[0]
        Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)

        manip = self.robot.SetActiveManipulator('left_wam') # set the manipulator to leftarm
        # self.robot.ikmodel # if needed
  
        import IPython
        IPython.embed()

        with openravepy.Environment(): # lock environment
           sol = manip.FindIKSolution(Tgrasp, IkFilterOptions.CheckEnvCollisions) 
           print "sol is", sol

        grasp_config=sol
        # base_pose is not finished
        basemanip = openravepy.interfaces.BaseManipulation(self.robot)
        basemanip.MoveToHandPosition(matrices=[Tgrasp])
        
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


    # copy from hw1, order the grasps - call eval grasp on each, set the 'performance' index, and sort
    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        sigma_max=0
        volume_max=0
        for grasp in self.grasps_ordered:
          score2 = self.eval_grasp(grasp)
          if score2[0] > sigma_max:
            sigma_max = score2[0]
          if score2[1] > volume_max:
            volume_max = score2[1]

        # print sigma_max 

        for grasp in self.grasps_ordered:
          score2 = self.eval_grasp(grasp)
          min_sigma = (float(score2[0])/sigma_max) 
          volume = (float(score2[1])/volume_max) 
          score=0.8*min_sigma+0.2*volume
          grasp[self.graspindices.get('performance')] = score
        # sort!
        order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        print order
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
      with self.robot:
        #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
        try:
          contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)
          obj_position = self.gmodel.target.GetTransform()[0:3,3]
          # for each contact
          G = np.array([]) #the wrench matrix
          for c in contacts:
            pos = c[0:3] - obj_position
            dir = -c[3:] #this is already a unit vector
            #TODO fill G
            dir=np.array(dir)
            w=np.append(dir,np.cross(pos,dir))
            G=np.append(G,w)
          #TODO use G to compute scrores as discussed in class
          volume = 0
          nsize=len(G)
          G=G.reshape(nsize/6,6)
          G=G.transpose()
          s,v,d=np.linalg.svd(G)
          #from N's version
          min_sigma = min(v)
          G1 = G.transpose()
          A = np.dot(G,G1)
          B = np.linalg.det(A)
          if B >= 0:
            volume = math.sqrt(B)
          else: 
            volume=0
          sc=np.array([min_sigma, volume])
        #  print sc
          return sc #change this
        except openravepy.planning_error,e:
          #you get here if there is a failure in planning
          #example: if the hand is already intersecting the object at the initial position/orientation
          return  [0.00,0.00] # TODO you may want to change this