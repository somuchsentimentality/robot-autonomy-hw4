import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

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
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
	print self.lower_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

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

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)
	    #print start_config

            # TODO: Here you will construct a set of actionson
            #  to be used during the planning process
            #

            # Since an action is composed of only 3 variables (left, right, and duration),
            # It is not unreasonable to do a comprehensive action generation.
            omega_range = 1; # min/max velocity
            resolution = 0.1;
            n_pts = int(omega_range * 2 / resolution);

            omega = numpy.linspace(-omega_range, omega_range, n_pts)
            duration = 1 # Can make this a range for even more options

            # Generate all combinations of left and right wheel velocities
            for om_1 in omega:
                for om_2 in omega:
                    ctrl = Control(om_1, om_2, duration)
                    footprint = self.GenerateFootprintFromControl(start_config, ctrl, stepsize=0.01)
                    this_action = Action(ctrl, footprint)

                    self.actions[idx].append(this_action)
         
            

    def GetSuccessors(self, node_id):

        successors = {}

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        
	
	currentConfiguration = self.discrete_env.NodeIdToConfiguration(node_id)
	currentCoord = self.discrete_env.NodeIdToGridCoord(node_id)
	#print currentConfiguration
	#print currentCoord
	#print currentCoord[2]
	for action in self.actions[currentCoord[2]]:
	    
	    successorNodeid = self.discrete_env.ConfigurationToNodeId(currentConfiguration + action.footprint[len(action.footprint)-1])
	    successors[successorNodeid] = action
	#print successors
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

	start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        
        dist = numpy.linalg.norm(start_config - end_config)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
	# Heuristic Cost in uncertain for this case with three resolution,
	# I will update this part later

	start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_coord = self.discrete_env.NodeIdToGridCoord(goal_id)
	
	cost = 0
	for i in range(len(start_coord)):
	    cost = cost + abs(goal_coord[i]-start_coord[i])*self.discrete_env.resolution[i]
	    
	#cost = cost*self.discrete_env.resolution
        
        return cost

    def PrintActions(self):
        # for key in self.actions.keys():
        key = 0;  # Just choose one key for sake of example
        for action in self.actions[key]:
            c = action.control
            print "(%.2f %.2f) %.2f s" % (c.ul, c.ur, c.dt)

    def PlotEdge2(self, sconfig, econfig, color, size):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=size)
