import numpy, operator
from RRTPlanner import RRTTree
import copy

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan1 = []
        plan2 = []
        temp_start_config = copy.deepcopy(start_config)
        temp_goal_config = copy.deepcopy(goal_config)
        back_trace1 = []
        back_trace1.append(0)
        back_trace2 = []
        back_trace2.append(0)
        plan1.append(start_config)
        plan2.append(goal_config)
        while 1:
            if self.planning_env.Extend(temp_start_config, temp_goal_config) != None:
                break

            end_config = self.planning_env.GenerateRandomConfiguration()
            neighbor = []
            if len(plan1) > 1:
                for i in range(len(plan1)):
                    neighbor.append(self.planning_env.ComputeDistance(plan1[i], end_config))
                z1 = neighbor.index(min(neighbor))    
                temp_start_config = copy.deepcopy(plan1[z1])
                #back_trace1.append(z1)  
                del neighbor    
            if self.planning_env.Extend(temp_start_config, end_config) != None:
                if len(plan1) > 1:
                    back_trace1.append(z1)
                plan1.append(end_config)
                temp_start_config = copy.deepcopy(end_config)  

            end_config = self.planning_env.GenerateRandomConfiguration()
            neighbor = []
            if len(plan2) > 1:
                for i in range(len(plan2)):
                    neighbor.append(self.planning_env.ComputeDistance(plan2[i], end_config))
                z2 = neighbor.index(min(neighbor))    
                temp_goal_config = copy.deepcopy(plan2[z2])
                #back_trace2.append(z2)  
                del neighbor    
            if self.planning_env.Extend(temp_goal_config, end_config) != None:
                if len(plan2) > 1:
                    back_trace2.append(z2)
                plan2.append(end_config)
                temp_goal_config = copy.deepcopy(end_config) 

         

        plan.append(start_config)
        y = len(plan1)-1        
        while y != 0 :        
            plan.insert(1,plan1[y])
            y = copy.deepcopy(back_trace1[y-1])
        l = len(plan)    

        y = len(plan2)-1
        while y != 0 :
            plan.append(plan2[y])
            y = copy.deepcopy(back_trace2[y-1])          
        plan.append(goal_config)

#--------- Visualize for PR2 -------------------------
        if len(start_config) == 2:
            if len(back_trace1) > len(back_trace2):
                ul = len(back_trace1)
            else:
                ul = len(back_trace2)        

            for i in range(ul):
                if i < len(back_trace1):
                    self.planning_env.PlotEdge(plan1[back_trace1[i]],plan1[i+1])
                if i < len(back_trace2):
                    self.planning_env.PlotEdge(plan2[back_trace2[i]],plan2[i+1]) 
            self.planning_env.PlotEdge(temp_start_config, temp_goal_config) 
#--------- For Calculating Path Length ----------------

        dist = 0
        for i in range(len(plan)-2):
            dist = dist + self.planning_env.ComputeDistance(plan[i],plan[i+1])    
        print dist                         

        return plan
