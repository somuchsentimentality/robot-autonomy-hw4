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
	if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        Queue= []
        visitedQueue=[]
        dictionary={}
        costQueue=[]
        startPoint=self.discrete_env.ConfigurationToNodeId(start_config)
        goalPoint=self.discrete_env.ConfigurationToNodeId(goal_config)
	#print goalPoint
	
        Queue.append(startPoint)
        costQueue.append([self.planning_env.ComputeHeuristicCost(startPoint, goalPoint),startPoint])
        costQueueElement=costQueue.pop(0)
        visitedQueue.append(costQueueElement[1])
        costQueue.append([0,startPoint])
	print costQueueElement[1]
	print goalPoint
	
        while costQueueElement[1]!=goalPoint:
            if len(Queue)==0:
                plan=[]
                

	    successor=self.planning_env.GetSuccessors(costQueueElement[1])
            for i in successor:
		if self.visualize:
                    self.planning_env.PlotEdge2(self.discrete_env.NodeIdToConfiguration(i), self.discrete_env.NodeIdToConfiguration(costQueueElement[1]), "k", 1.5)
		actionlist[i] = successor[i]
                if i not in visitedQueue and i not in Queue:        
                    Queue.append(i)
                    dictionary[i]=costQueueElement[1]
                    costG=0
                    X=i
                    while X!=startPoint:
			costG=costG+self.planning_env.ComputeDistance(X, dictionary[X])
                        X=dictionary[X]
                    heurPlusG=self.planning_env.ComputeHeuristicCost(i, goalPoint) + costG
                    costQueue.append([heurPlusG,i]) 
	    #print costQueue
            costQueue=sorted(costQueue)
            costQueueElement=costQueue.pop(0)
            Queue.remove(costQueueElement[1])
	    visitedQueue.append(costQueueElement[1])
	
	if self.visualize:
	    pl.draw()
	duration = time.clock() - starttime
        print "Goal Found"
	
        point=goalPoint
	pathlength = 0
        while point!=startPoint:
            pointPlan.append(self.discrete_env.NodeIdToConfiguration(point))
	    pathlength = pathlength + self.planning_env.ComputeDistance(point, dictionary[point])
            point=dictionary[point]
	   
        pointPlan.append(start_config)
        #pointPlan.reverse()
	
	for i in pointPlan:
	    idx = self.discrete_env.ConfigurationToNodeId(i)
	    plan.append(actionlist[idx])

	plan  = numpy.array(plan)
	    
	print "Time =", duration, "s"
	print "Nodes popped =", len(visitedQueue)
	print "Path length =", pathlength, "m"
        return plan
