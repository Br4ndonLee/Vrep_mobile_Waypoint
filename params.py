## VREP Simulation parameters:


## Perception Parameters:



## Planning parameters:

# a. Planner grid parameters:
# Parameters:
# Width of Grid in X-direction : widthX
# Width of Grid in Y-direction : widthY
# Resolution of the grid: gridRes
# Width of the robot (A square for now) - robotWidth
widthX = 7.0
widthY = 7.0
gridRes = 0.5
robotWidth = 0.45


# Grid co-ordinates: Initialized later
X = None
Y = None

# Initialized here only during testing phase, else commeneted out: Obtain from VREP or real world: Buck up!
# seg1 = [(7,0) ,(2.5,7.5)]
# seg2 = [(5,20) ,(12.5,5)]
# seg3 = [(20,10), (15,10)]
#
# mazeSegments = [seg1, seg2, seg3]

mazeSegments = None

# b. Dijkstra parameters:


# c. A* parameters:
# Weighting factor for heurestic - epsilon
epsilon = 0.0

# d. RRT parameters:
windowSize = 7.0 # The operating window for each D.O.F
numNodes = 500 # Maximum number of nodes
stepSize= 0.5 # For the tree growth step
GG = 50

# Inflated maze segments:
infMazeSegments = []

# Storage of co-ods of maze segments:
mxVec , myVec , nxVec , nyVec = None , None , None, None

# Storage for x and y co-od sof nodes in the tree:
pxVec = []
pyVec = []

# Nodes on the Random Tree:
nodes = []


## Controller parameters:
# Proportional gain for go-to-goal controller for orientation - kpTheta
# Proportional gain for go-to-goal controller for distance - kpDist
# Derivative gain for go-to-goal controller for orientation - kdTheta
# Derivative gain for go-to-goal controller for distance - kdDist
# Integral gain for go-to-goal controller for orientation - kiTheta
# Integral gain for go-to-goal controller for distance - kiDist
# Frequency of controller - freqCont - (In Hz)
# Threshold on the angular error - angErrThresh - radians
# Threshold on the distance error - distErrThresh - Parameters
# Window for Integral control: integralWindow - Set to -1 if entire error history is to be used - Need to implement this feature:


kpTheta = 0.3
# kpDist = 0.08
kpDist = 0.1

kiTheta = 0.005
# kiDist = 0.02
kiDist = 0

# kdTheta = 0.02
kdTheta = 0.2
# kdDist = 0.008
kdDist = 0.01

angErrThresh = 0.01
distErrThresh = 0.2

integralWindow = 8


distSlope = 5
angSlope = 0.05
