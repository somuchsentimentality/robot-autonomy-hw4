import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import time, operator, numpy

class Node:
    def __init__(self, id, gcost, hcost=0):
        self.id = id
        self.parent_id = None
        self.parent_to_me_action = None
        self.gcost = gcost
        self.hcost = hcost
        self.fcost = gcost + hcost

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
       
	self.discrete_env = DiscreteEnvironment(self.planning_env.resolution,self.planning_env.lower_limits, self.planning_env.upper_limits)

    def Plan(self, start_config, goal_config):

        plan = []
        totallen = 1

        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        discrete_env = self.planning_env.discrete_env

        start_config_id = discrete_env.ConfigurationToNodeId(start_config)
        goal_config_id = discrete_env.ConfigurationToNodeId(goal_config)
        start_config = Node(start_config_id,0)
        nodes = {}
        parent = {}
        queue = []
        visited = []

        visited.append(start_config.id)
        nodes[start_config.id] = start_config
        queue.append(start_config)

        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        while queue:
            node = queue.pop(0)
            totallen = totallen + 1

            if node.id == goal_config_id:
                path = [nodes[goal_config_id]] # list of nodes            
                while path[-1].parent_id != start_config_id:
                    try:
                        path.append(nodes[path[-1].parent_id])
                    except KeyError:
                        print "AStarPlanner: Path KeyError"
                        import IPython
                        IPython.embed()
                path.reverse()  
                break
                
            Successors_id, Actions = self.planning_env.GetSuccessors(node.id)
            Successors = [Node(s,0) for s in Successors_id]
            # import IPython
            # IPython.embed()
            for i in range(len(Successors_id)):
                adjacent = Successors[i]
                if adjacent.id not in visited:
                    # import IPython
                    # IPython.embed()
                    visited.append(adjacent.id)
                    adjacent.parent_to_me_action = Actions[i]
                    adjacent.gcost = node.gcost + self.planning_env.ComputeDistance(node.id, adjacent.id)
                    adjacent.hcost = self.planning_env.ComputeHeuristicCost(adjacent.id, goal_config_id)        
                    adjacent.fcost = adjacent.gcost + adjacent.hcost
                    
                    adjacent.parent_id = node.id
                    parent[adjacent.id] = node.id # <<<<< record its parent 
                    nodes[adjacent.id] = adjacent

                    queue.append(adjacent)

                    if self.visualize:
                        node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node.id)
                        adjacent_config = self.planning_env.discrete_env.NodeIdToConfiguration(adjacent.id)
                        # print "Adjacent: %r" % adjacent_config
                        self.planning_env.PlotEdge(node_config, adjacent_config)
                        # raw_input("Added successor")

            queue.sort(key=operator.attrgetter('fcost'))        #sort class objects with respect to fcost

        plan = [node.parent_to_me_action for node in path]   
        print plan            
        # plan.append(start_config)
        # plan.append(goal_config)

        print 'number nodes', totallen
        return plan

