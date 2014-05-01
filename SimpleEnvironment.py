import numpy, openravepy, math
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

	#For Collision Check, add table
	self.env = self.robot.GetEnv()
        # self.table = self.env.GetBodies()[1]

        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
	#print self.lower_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.resolution = resolution
        self.ConstructActions()

        self.PlotActionFootprints(0)
        
        raw_input("Showing action footprints, hit enter to continue")

        # import IPython
        # IPython.embed()
	

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
            omega_range = 0.5; # min/max velocity
            resolution = 0.1;
            n_pts = int(omega_range * 2 / resolution);

            omega = numpy.linspace(-omega_range, omega_range, n_pts)
            duration = 1 # Can make this a range for even more options

            controls = []
            # controls.append(om1, om2, duration)
            controls.append(Control(0.5,0.5,1.0))
            controls.append(Control(1.0,1.0,1.0))
            controls.append(Control(1.0,1.0,4.0))
            controls.append(Control(-0.5,0.5,1))
            controls.append(Control(0.5,-0.5,1))
            controls.append(Control(0.5,1.0,4.0))
            controls.append(Control(1.0,0.5,4.0))


            for ctrl in controls:
                footprint = self.GenerateFootprintFromControl(start_config, ctrl, stepsize=0.01)
                this_action = Action(ctrl, footprint)
                # print this_action
                self.actions[idx].append(this_action)
           
    def GetSuccessors(self, node_id):
        successors = []
        actions_returned = []

        save_Transform = self.robot.GetTransform()
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodesc
        currentConfiguration = self.discrete_env.NodeIdToConfiguration(node_id)
        currentCoord = self.discrete_env.NodeIdToGridCoord(node_id)
	
        print "num(actions):%r, currentCoord:%r" % (len(self.actions), currentCoord)

        for action in self.actions[currentCoord[2]]:
            end_fp_config = currentConfiguration + action.footprint[len(action.footprint)-1]
            
            angle = end_fp_config[2]
            save_angle = angle
            while angle > numpy.pi:
                angle -= 2*numpy.pi 
            while angle < -numpy.pi:
                angle += 2*numpy.pi

            end_fp_config[2] = angle

            successorNodeid = self.discrete_env.ConfigurationToNodeId(end_fp_config)
	        
            # For each action check whether generated footprint is collision free
            inBound = not ((end_fp_config < self.lower_limits).any() or (end_fp_config > self.upper_limits).any())
            
            self.robot.SetTransform(self.determineThePose(end_fp_config))
            
            collisionfree = True
            for body in self.env.GetBodies():
                if self.env.CheckCollision(self.robot, body):
                    collisionfree = False
                    # print "Successor in collision!"
                    break

            has_collision = False
            for fp in action.footprint:
                fp_config = fp.copy()
                fp_config[0] += end_fp_config[0]
                fp_config[1] += end_fp_config[1]
                if (not (inBound and collisionfree)):
                    has_collision = True
                    break
            if (not has_collision):
                successors.append(successorNodeid)
                actions_returned.append(action)


        self.robot.SetTransform(save_Transform)        
        return successors, actions_returned

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

	start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        end_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        
        dist = numpy.linalg.norm(start_config[:2] - end_config[:2])

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
    
        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_coord = self.discrete_env.NodeIdToGridCoord(goal_id)

        grid_distance = self.GetConfigXYDistance(start_id, goal_id)

        cost = 0

        if grid_distance > 3: 
            # If we are far away from our goal in xy
            # then don't worry about the orientation
            for i in range(len(start_coord)-1):
                diff = abs(goal_coord[i]-start_coord[i])
                cost += diff**2 
        else:
            # If we are close to our goal, look at orientation
            for i in range(len(start_coord)):
                diff = abs(goal_coord[i]-start_coord[i])
                cost += diff**2 

        return math.sqrt(cost)

    def GetConfigXYDistance(self,nodeID_1, nodeID_2):
        config_1 = self.discrete_env.NodeIdToConfiguration(nodeID_1)
        config_2 = self.discrete_env.NodeIdToConfiguration(nodeID_2)
        paired = zip(config_1, config_2)

        diff_sqrd = [(x[0] - x[1])*(x[0] - x[1]) for x in paired]

        dist = math.sqrt(diff_sqrd[0] + diff_sqrd[1]) # only use x,y coords
        return dist

    def PrintActions(self):
        # for key in self.actions.keys():
        key = 0;  # Just choose one key for sake of example
        for action in self.actions[key]:
            c = action.control
            print "(%.2f %.2f) %.2f s" % (c.ul, c.ur, c.dt)

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig):
        self.PlotEdge2(sconfig, econfig, 'k', 2)

    def PlotEdge2(self, sconfig, econfig, color, size):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=size)


    def determineThePose (self, config):
	pose = self.robot.GetTransform()
        pose[0:3,3] = numpy.array([config[0], config[1], 0.0])
        a = config[2]
        rotation = numpy.array([[numpy.cos(a), -numpy.sin(a),0.0],[numpy.sin(a),numpy.cos(a),0.0],[0.0,0.0,1.0]])
        pose[0:3,0:3] = rotation
	return pose

    
