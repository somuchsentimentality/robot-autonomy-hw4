import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import time
import numpy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
       
	self.discrete_env = DiscreteEnvironment(self.planning_env.resolution,self.planning_env.lower_limits, self.planning_env.upper_limits)


    def Plan(self, start_config, goal_config):

	print "Searching..."
	
	starttime = time.clock()
        pointPlan = []
	plan = []
	actionlist = {}
	dictionary={}
	if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        Queue= []
        visitedQueue=[]
        costQueue=[]
        startPoint=self.discrete_env.ConfigurationToNodeId(start_config)
        goalPoint=self.discrete_env.ConfigurationToNodeId(goal_config)
	#print goalPoint
	
        Queue.append(startPoint)
        costQueue.append([self.planning_env.ComputeHeuristicCost(startPoint, goalPoint),startPoint])
        costQueueElement=costQueue.pop(0)
        visitedQueue.append(costQueueElement[1])
        costQueue.append([0,startPoint])

        while costQueueElement[1]!=goalPoint:
            if len(Queue)==0:
                plan=[]
       
	    successor = self.planning_env.GetSuccessors(costQueueElement[1]);

	    for j in successor:
		if j not in actionlist.keys():
		    actionlist[j] = successor[j] 

            for i in successor:
		if self.visualize:
                    self.planning_env.PlotEdge2(self.discrete_env.NodeIdToConfiguration(i), self.discrete_env.NodeIdToConfiguration(costQueueElement[1]), "k", 1.5)
                    raw_input("Popped node, enter to continue")
                if i not in visitedQueue and i not in Queue:        
                    Queue.append(i)
                    dictionary[i]=costQueueElement[1]
                    costG=0
                    X=i
                    while X!=startPoint:
			costG=costG+self.planning_env.ComputeDistance(X, dictionary[X])
                        X=dictionary[X]
                    F = self.planning_env.ComputeHeuristicCost(i, goalPoint)
                    heurPlusG= 0.7*F + 0.3*costG
                    costQueue.append([heurPlusG,i]) 
            costQueue=sorted(costQueue)
            costQueueElement=costQueue.pop(0)
            Queue.remove(costQueueElement[1])
	    visitedQueue.append(costQueueElement[1])
	
	if self.visualize:
	    pl.draw()
	duration = time.clock() - starttime
        print "Goal Found"
	
	self.planning_env.robot.SetTransform(self.planning_env.determineThePose(start_config))
        point=goalPoint
	pathlength = 0
	
        while point!=startPoint:
            pointPlan.append(self.discrete_env.NodeIdToConfiguration(point))
	    pathlength = pathlength + self.planning_env.ComputeDistance(point, dictionary[point])
            point=dictionary[point]
	    
	   
        pointPlan.append(start_config)
	#plan.append([actionlist[self.discrete_env.ConfigurationToNodeId(start_config)]])
        pointPlan.reverse()
	
	
	
	for i in pointPlan[1:]:
	    idx = self.discrete_env.ConfigurationToNodeId(i)
	    plan.append(actionlist[idx])
	plan.append(actionlist[goalPoint])
	plan.reverse()
	plan  = numpy.array(plan)
	    
	print "Time =", duration, "s"
	print "Nodes popped =", len(visitedQueue)
	print "Path length =", pathlength, "m"
        return plan

    def GetConfigXYDistance(self,nodeID_1, nodeID_2):
        config_1 = self.discrete_env.NodeIdToConfiguration(nodeID_1)
        config_2 = self.discrete_env.NodeIdToConfiguration(nodeID_2)
        paired = zip(config_1, config_2)

        diff_sqrd = [(x[0] - x[1])*(x[0] - x[1]) for x in paired]

        dist = math.sqrt(diff_sqrd[0] + diff_sqrd[1]) # only use x,y coords
        return dist