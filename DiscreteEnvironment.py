import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution
        #print resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        print "Limits, Lower: %r, Upper: %r" % (lower_limits, upper_limits)
        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])
        #print self.num_cells
        
    def ConfigurationToNodeId(self, config):
            
            # TODO:
            # This function maps a node configuration in full configuration
            # space to a node in discrete space
            #
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id
        
    def NodeIdToConfiguration(self, nid):
            
            # TODO:
            # This function maps a node in discrete space to a configuraiton
            # in the full configuration space
            #
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config

    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        i=0
        coord = [0] * self.dimension
        while i<len(config):
            this_coord = numpy.floor( (config[i] - self.lower_limits[i])  / self.resolution[i])
            coord[i] = self.clamp(this_coord, 0, self.num_cells[i]-1)
            i = i+1
        return coord

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        i=0
        half = [x / 2 for x in self.resolution]
        while i < len(coord):
            config[i]=(coord[i] * self.resolution[i]) + self.lower_limits[i] + half[i]
            i = i+1
        return numpy.array(config)

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id
        i=0
        total_dims = 1
        node_id = coord[0]
        while i<len(coord)-1:
            total_dims = total_dims * self.num_cells[i]
            node_id = node_id + (coord[i+1] * total_dims)
            i = i+1
        return int(node_id)

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        total_dims = 1
        for i in xrange(self.dimension-1):
            total_dims = total_dims * self.num_cells[i]
        
        for k in xrange(self.dimension-1, -1, -1):
            coord[k] =  int(node_id)/int(total_dims)
            node_id = node_id - (coord[k]*total_dims)
            total_dims = total_dims/self.num_cells[k-1]
        return coord

## def round(float):

'''
print
print "test = DiscreteEnvironment(0.2, [0,0,0], [1,1,1])"
test = DiscreteEnvironment(0.2, [0,0,0], [1,1,1])
print "testing ConfigurationToGridCoord([0.26, 0.64,0.6])... "
print test.ConfigurationToGridCoord([0.26, 0.64,0.6])
print
print "testing ConfigurationToNodeId([0.26,0.64,0.6])... "
print test.ConfigurationToNodeId([0.26,0.64,0.6])
print
print "testing NodeIdToConfiguration(66)... "
print test.NodeIdToConfiguration(66)
print "testing GridCoordToConfiguration([1,3,2])..."
print test.GridCoordToConfiguration([1,3,2])
print
print "testing GridCoordToNodeId([1,3,2])..."
print test.GridCoordToNodeId([1,3,2])
print
'''

        
        
