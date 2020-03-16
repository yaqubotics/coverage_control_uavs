import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import compute_coverage as com
import load_parameter as lp

class NoEstimation:
	# structure related to the Recursive Estimation
	Node = com.nodeInit(lp.Parameter.Positions,lp.Parameter)
	
	# initial density function
	DensityFunction = np.ones((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))
	
	# initial posterior
	APosterioriVariance = np.ones((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))
	#APosterioriVarianceLists = cell(1,lp.Parameter.Niter)
	# energy at each iteration
	averageEnergy = [] #np.zeros((lp.Parameter.Niter,1))
	
	# evolution of the cost function
	CostFunction = [] #np.zeros((lp.Parameter.Niter,1))
	
	# compute Voronoi Regions for Only Coverage
	Node = com.computeVoronoiRegion(Node,lp.Parameter,lp.Parameter.ws_model)
	
	# compute mass, first momentum and centroid of the regions for Only Coverage
	print 'No Estimation'
	#print lp.TrueFunction(lp.Parameter)
	Node = com.computeCentroid(Node,lp.Parameter,lp.TrueFunction(lp.Parameter),lp.Parameter.ws_model)
	#print Node

	
class RecursiveEstimation:
	# structure related to the Recursive Estimation
	Node = com.nodeInit(lp.Parameter.Positions,lp.Parameter)
	
	# initial density function
	DensityFunction = np.ones((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))
	MeanErrorDensityFunction = []
	# initial posterior
	APosterioriVariance = np.ones((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))
	APosterioriVariancePure = np.ones((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))

	#APosterioriVarianceLists = cell(1,lp.Parameter.Niter)
	# energy at each iteration
	averageEnergy = [] #np.zeros((lp.Parameter.Niter,1))
	
	# structure for just coverage and no estimation
	#def NoEstimation(self):
	#	return self
	
	# max of the posterior at each iteration
	maxAPosterioriVariance = [] #np.ones((lp.Parameter.Niter,1))
	
	# average of the posterior at each iteration
	minAPosterioriVariance = [] #np.ones((lp.Parameter.Niter,1))
	
	# min of the posterior at each iteration
	averageAPosterioriVariance = [] #np.ones((lp.Parameter.Niter,1))

	maxAPosterioriVarianceVoronoi = np.zeros((lp.Parameter.NumberOfPoints,lp.Parameter.NumberOfPoints))
	listMaxAPosterioriVarianceVoronoi = []
	for i in range(lp.Parameter.NumberOfAgents):
		listMaxAPosterioriVarianceVoronoi.append(maxAPosterioriVarianceVoronoi)

	statusFinishedAgent = []
	for i in range(lp.Parameter.NumberOfAgents):
		statusFinishedAgent.append(0)
	# evolution of the cost function
	CostFunction = [] #np.zeros((lp.Parameter.Niter,1))
	
	#energy function
	EnergyFunction = [] #np.zeros((lp.Parameter.Niter,1))
	
	# all the input locations
	InputLocations = None
	
	# all the direct path
	DirectPaths = []
	
	# Number of agents which are making exploration
	NumberOfAgentsExploring = [] #np.zeros((1,lp.Parameter.Niter))
	NumberOfAgentsExploring.append(lp.Parameter.NumberOfAgents)
	
	# compute Voronoi Regions for Recursive Estimation
	#print lp.Parameter.VoronoiMode
	Node = com.computeVoronoiRegion(Node,lp.Parameter,lp.Parameter.ws_model)
	#print Node[7].VoronoiRegion
	# compute mass, first momentum and centroid of the regions for Recursive Estimation
	print 'Recursive Estimation'
	Node = com.computeCentroid(Node,lp.Parameter,DensityFunction,lp.Parameter.ws_model)
	
	MaxofTheMaxVariance = 0
	# Saving the NoEstimation Structure in Parameter
	#lp.Parameter.NoEstimation = NoEstimation
	
	# setting this variable to 'Centroid' the voronoi will be computed wrt Centroid
	# using 'Positions' will be computed wrt the positions of the agents
	#lp.Parameter.VoronoiMode = 'Centroid'
	
	#lp.Parameter.KbarLists = cell(1,lp.Parameter.Niter)
	#lp.Parameter.AutocovarianceLists = cell(1,lp.Parameter.Niter)
	lastKbar = np.zeros((1,1)) #np.zeros((lp.Parameter.NumberOfAgents,lp.Parameter.NumberOfPoints*lp.Parameter.NumberOfPoints))
	
# Saving the NoEstimation Structure in Parameter
#lp.Parameter.NoEstimation = NoEstimation
lp.Parameter.VoronoiMode = 'Centroid'#'Positions' #
'''
par = Parameter()
fig = plt.figure()
ax = fig.gca(projection='3d')
print par.Xgrid.shape
print par.Ygrid.shape
print par.TrueFunction().shape
# Plot the surface.
surf = ax.plot_surface(par.Xgrid, par.Ygrid, par.TrueFunction(), cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
'''

